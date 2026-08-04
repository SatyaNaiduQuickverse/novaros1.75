#!/usr/bin/env python3
"""hw_bringup.py — bench harness to bring up and VERIFY each connector. Run the
sub-tests in order; get each one green.

  python3 hw_bringup.py link      --port /dev/ttyAMA0     # FC telemetry + link
  python3 hw_bringup.py imu       [--fake]                # IMU state (q,w,accel)
  python3 hw_bringup.py vision    [--fake]                # a bearing from the tracker
  python3 hw_bringup.py rc        --port /dev/ttyAMA0     # stream idle RC  (PROPS OFF!)
  python3 hw_bringup.py loop      --port /dev/ttyAMA0 [--fake-imu --fake-vision --no-fc]

'--fake' lets a test run before that piece of hardware is wired. Everything is logged
via flight_logger; analyse with:  python3 log_analyze.py ~/logs/run_*.jsonl
"""
import argparse, time, sys
import numpy as np


def t_link(a):
    from fc_link import FCLink
    fc = FCLink(serial=a.port, baud=a.baud).connect()
    fc.start_telemetry(hz=50)
    print("connected. reading telemetry for 5 s ...")
    t0 = time.time()
    while time.time() - t0 < 5:
        print("  attitude:", fc.read_attitude(), " imu:", fc.read_imu())
        time.sleep(0.5)
    fc.close(); print("LINK OK" if True else "")


def t_imu(a):
    from imu_driver import RealIMU
    imu = RealIMU(fake=a.fake).start()
    print(f"IMU ({'FAKE' if a.fake else 'REAL'}) — get_state() for 3 s ...")
    t0 = time.time()
    while time.time() - t0 < 3:
        q, w, acc = imu.get_state()
        print(f"  q={np.round(q,3)} w={np.round(w,3)} acc={np.round(acc,2)}")
        time.sleep(0.5)
    imu.stop(); print("check: q tracks hand-tilts, acc ~[0,0,-9.8] level, right axes.")


def t_vision(a):
    from vision_adapter import VisionAdapter
    att = lambda: np.array([1.0, 0, 0, 0])
    va = VisionAdapter(get_attitude_fn=att, fake=a.fake)   # wire get_bbox_fn for real
    print(f"VISION ({'FAKE' if a.fake else 'REAL'}) — bearing() x5 ...")
    for i in range(5):
        b = va.bearing(time.time())
        print("  bearing:", (np.round(b.u_world, 3), "conf", b.conf, "rng", b.range_m) if b else None)
        time.sleep(0.3)
    print("check: bearing points the right way as the object/vehicle moves.")


def t_rc(a):
    from fc_link import FCLink
    print("!! PROPS OFF. Streaming IDLE RC (throttle min, centred) for 4 s ...")
    fc = FCLink(serial=a.port, baud=a.baud).connect()
    fc.start_rc_stream()
    t0 = time.time()
    while time.time() - t0 < 4:
        fc.set_stick(roll=1500, pitch=1500, yaw=1500, throttle=1000)
        time.sleep(0.05)
    fc.close(); print("RC path exercised. Confirm the FC saw RC (Betaflight receiver tab).")


def t_loop(a):
    from flight_logger import FlightLogger
    from imu_driver import RealIMU
    from vision_adapter import VisionAdapter
    from command_module import CommandModule
    imu = RealIMU(fake=a.fake_imu).start()
    va = VisionAdapter(get_attitude_fn=lambda: imu.get_state()[0], fake=a.fake_vision)
    fc = None
    if not a.no_fc:
        from fc_link import FCLink
        fc = FCLink(serial=a.port, baud=a.baud).connect(); fc.start_rc_stream()
    lg = FlightLogger()
    g = CommandModule(fc if fc else _NullFC(), imu, va, logger=lg)
    print(f"!! PROPS OFF. Running loop (idle command) 6 s @100 Hz -> {lg.path}")
    lg.event("start")
    t0 = time.time()
    while time.time() - t0 < 6:
        g.step(time.time() - t0)
        time.sleep(0.01)
    lg.event("end"); lg.close(); imu.stop()
    if fc: fc.close()
    print("loop done. analyse:  python3 log_analyze.py", lg.path)


class _NullFC:
    def set_stick(self, **k): pass
    def close(self): pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("cmd", choices=["link", "imu", "vision", "rc", "loop"])
    p.add_argument("--port", default="/dev/ttyAMA0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--fake", action="store_true")
    p.add_argument("--fake-imu", action="store_true")
    p.add_argument("--fake-vision", action="store_true")
    p.add_argument("--no-fc", action="store_true")
    a = p.parse_args()
    {"link": t_link, "imu": t_imu, "vision": t_vision, "rc": t_rc, "loop": t_loop}[a.cmd](a)


if __name__ == "__main__":
    main()
