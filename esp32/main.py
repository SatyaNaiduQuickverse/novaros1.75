"""ESP32-C6 IMU bridge — MPU6500 over I2C, framed binary stream over USB CDC.

Runs on the ESP32-C6 (MicroPython). The IMU sits on the ESP32's I2C bus, not
the Pi's, so this is what gets the samples to the companion computer.

    Pi  <--USB CDC--  ESP32-C6  <--I2C 400kHz--  MPU6500
                      SCL GPIO19 / SDA GPIO20

Deliberately does NOT run an attitude filter. It ships raw counts and lets the
Pi fuse them: MicroPython on this chip cannot hold a solid 200 Hz Madgwick, the
Pi has numpy and CPU to spare, and keeping the filter on the Pi means it can be
changed without reflashing.

Frame, 18 bytes little-endian:

    A5 5A | seq u8 | ax ay az  TEMP  gx gy gz  (7 x int16) | xor u8

Temperature sits between the accelerometer and the gyro because that is the
MPU6500's own register order and this bridge burst-reads them verbatim. The
Pi-side unpacker matches; only this comment was ever wrong.

`seq` wraps at 256 and is how the Pi detects dropped samples. The trailing byte
is an XOR over seq+payload — USB CDC already guarantees integrity end to end,
so this exists to catch frame desync, not corruption.

Counts are raw sensor LSBs at the ranges configured below (accel +/-16 g,
gyro +/-2000 dps); scaling to SI happens on the Pi so both ends cannot disagree
about it silently.

Install as /main.py to run at boot.

⚠️ Read docs/ESP32_BRIDGE_FAULTS.md before touching this file. It records what
each defensive measure below is defending against, all of it measured.

⚠️ THE DEVICE IS RUNNING THE BUILD FROM BEFORE THE WDT/YIELD CHANGES BELOW.
   Reflashing needs a REPL and the running build cannot give one — see
   "not interruptible". Everything below is correct for the NEXT flash; do not
   read it as a description of the ESP32 currently on the bench.

**Not interruptible while streaming.** MEASURED 2026-08-04: once ``stream()``
is running, the host-to-device direction is dead — every write to the CDC
endpoint times out, so Ctrl-C never lands and STARTUP_DELAY_S buys nothing.
The 200 Hz loop never yields long enough for MicroPython's USB-Serial-JTAG RX
task to drain the OUT endpoint. ``time.sleep_ms(0)`` below is the fix, and
until this is reflashed the only ways in are the ROM download mode (esptool
holding GPIO9 low) or a full erase.

**Wedges if nothing reads.** Also measured: if no host drains the CDC endpoint
in the seconds after boot, the first ``out.write()`` blocks and the bridge
never recovers — it sits enumerated and silent indefinitely. A companion
reboot causes exactly this (the ESP32 came up 4 s before the Pi's cdc_acm
driver attached). The WDT below turns that into a reboot loop that self-heals
the moment a reader appears; ``companion.imu_esp32.hardware_reset`` is the
independent belt-and-braces from the Pi side.
"""

import struct
import sys
import time

from machine import I2C, Pin, WDT

# ---------------------------------------------------------------- wiring
SCL_PIN = 19
SDA_PIN = 20
I2C_FREQ = 400_000
I2C_BUS = 0

# ---------------------------------------------------------------- MPU6500
ADDR = 0x68
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CONFIG2 = 0x1D
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_PWR_MGMT_2 = 0x6C
REG_WHO_AM_I = 0x75

WHOAMI_MPU6500 = 0x70
WHOAMI_MPU9250 = 0x71

# Ranges. These MUST match companion/imu_esp32.py's scale constants.
GYRO_FS = 3          # 0=250 1=500 2=1000 3=2000 dps
ACCEL_FS = 3         # 0=2   1=4   2=8    3=16  g
DLPF = 3             # 41 Hz gyro bandwidth
ACCEL_DLPF = 3       # 41 Hz accel bandwidth
SAMPLE_RATE_HZ = 200
SMPLRT_DIV = 1000 // SAMPLE_RATE_HZ - 1     # base is 1 kHz with DLPF enabled

SYNC = b"\xA5\x5A"
# Wait this long for a host before streaming into a port nobody has opened.
# The Pi's cdc_acm driver only drains the endpoint once the tty is OPEN, so a
# bridge that boots first blocks on its very first write and stays wedged.
# MEASURED: the ESP32 came up 4 s ahead of the companion after a Pi reboot, so
# 2 s was nowhere near enough. This is a ceiling, not a fixed cost — any byte
# from the host ends it immediately (see _break_requested).
STARTUP_DELAY_S = 10.0
STATUS_LED = None    # set to a pin number if the board has one
# A blocked write is indistinguishable from a hang, and both are fatal to the
# attitude source. 4 s is ~800 sample periods: far too long to trip on a slow
# loop, far too short to sit silent through a flight.
WDT_TIMEOUT_MS = 4000


def mpu_init(i2c):
    """Reset and configure the IMU. Raises if the part is not what we expect."""
    who = i2c.readfrom_mem(ADDR, REG_WHO_AM_I, 1)[0]
    if who not in (WHOAMI_MPU6500, WHOAMI_MPU9250):
        raise OSError("unexpected WHO_AM_I 0x%02X at 0x%02X" % (who, ADDR))

    i2c.writeto_mem(ADDR, REG_PWR_MGMT_1, b"\x80")   # device reset
    time.sleep_ms(100)
    i2c.writeto_mem(ADDR, REG_PWR_MGMT_1, b"\x01")   # wake, auto-select clock
    time.sleep_ms(10)
    i2c.writeto_mem(ADDR, REG_PWR_MGMT_2, b"\x00")   # all axes enabled
    i2c.writeto_mem(ADDR, REG_CONFIG, bytes([DLPF]))
    i2c.writeto_mem(ADDR, REG_GYRO_CONFIG, bytes([GYRO_FS << 3]))
    i2c.writeto_mem(ADDR, REG_ACCEL_CONFIG, bytes([ACCEL_FS << 3]))
    i2c.writeto_mem(ADDR, REG_ACCEL_CONFIG2, bytes([ACCEL_DLPF]))
    i2c.writeto_mem(ADDR, REG_SMPLRT_DIV, bytes([SMPLRT_DIV]))
    time.sleep_ms(20)
    return who


def stream(i2c, wdt=None):
    out = sys.stdout.buffer
    buf = bytearray(14)
    frame = bytearray(18)
    frame[0:2] = SYNC
    seq = 0
    period_us = 1_000_000 // SAMPLE_RATE_HZ
    next_us = time.ticks_us()

    while True:
        # Burst-read accel(6) + temp(2) + gyro(6) so all axes share a sample
        # instant; reading them separately would smear them across the bus.
        i2c.readfrom_mem_into(ADDR, REG_ACCEL_XOUT_H, buf)

        frame[2] = seq
        # Sensor is big-endian, the wire format is little-endian: swap here so
        # the Pi can unpack with a plain '<7h'.
        for i in range(7):
            frame[3 + 2 * i] = buf[2 * i + 1]
            frame[4 + 2 * i] = buf[2 * i]
        x = 0
        for i in range(2, 17):
            x ^= frame[i]
        frame[17] = x
        out.write(frame)
        # Only fed AFTER the write returns: a write that blocks because no
        # host is draining the endpoint is precisely what must trip the
        # watchdog, and feeding first would make the bridge hang politely.
        if wdt is not None:
            wdt.feed()

        seq = (seq + 1) & 0xFF
        next_us = time.ticks_add(next_us, period_us)
        delay = time.ticks_diff(next_us, time.ticks_us())
        if delay > 0:
            # sleep_ms(0) yields to the scheduler so MicroPython's
            # USB-Serial-JTAG RX task gets to drain the OUT endpoint; without
            # it this loop starves it and Ctrl-C can never land.
            time.sleep_ms(0)
            delay = time.ticks_diff(next_us, time.ticks_us())
            if delay > 0:
                time.sleep_us(delay)
        else:
            next_us = time.ticks_us()   # fell behind; resync rather than spiral


def _wait_for_host(seconds):
    """Wait up to `seconds` for the host, and report what it asked for.

    Returns "break" if it sent Ctrl-C (0x03) — drop to the REPL and, crucially,
    do NOT start the watchdog, because an ESP32 WDT cannot be switched off once
    running and would make this board unreflashable except via esptool.
    Returns "go" for any other byte: a host is listening, start now rather than
    sitting out the rest of the timeout. Returns "timeout" if nothing came.

    Polling rather than relying on KeyboardInterrupt is deliberate: Ctrl-C only
    fires if the RX path got serviced, and the RX path not being serviced is
    the exact fault this whole function exists to work around.
    """
    try:
        import select
        poller = select.poll()
        poller.register(sys.stdin, select.POLLIN)
        deadline = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            if poller.poll(50):
                return "break" if sys.stdin.read(1) == "\x03" else "go"
    except KeyboardInterrupt:
        return "break"
    except Exception:
        # No pollable stdin on this port/build — fall back to a plain wait so
        # the bridge still starts. It just cannot be signalled this way.
        time.sleep(seconds)
    return "timeout"


def main():
    # Do not stream until a host is listening — see STARTUP_DELAY_S.
    asked = _wait_for_host(STARTUP_DELAY_S)
    if asked == "break":
        print("bridge: break requested, dropping to REPL (watchdog not started)")
        return
    wdt = WDT(timeout=WDT_TIMEOUT_MS)
    while True:
        try:
            i2c = I2C(I2C_BUS, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)
            mpu_init(i2c)
            stream(i2c, wdt)
        except KeyboardInterrupt:
            print("\nstopped")
            return
        except Exception as e:
            # A loose I2C wire must not brick the bridge — report on stderr
            # (which the Pi-side parser ignores) and retry. Feed the watchdog
            # while retrying: rebooting does not reconnect a wire, and the Pi
            # resets us anyway once no frames arrive.
            sys.stderr.write("imu error: %r\n" % (e,))
            wdt.feed()
            time.sleep(1)


if __name__ == "__main__":
    main()
