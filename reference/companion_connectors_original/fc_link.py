import struct, time, threading
MSP_STATUS_EX = 150
MSP_RAW_IMU = 102
MSP_ATTITUDE = 108
MSP_SET_RAW_RC = 200
ACC_PER_G_SITL = 256.0
G = 9.80665
GYRO_SCALE_DPS = 1.0 / 16.4

def _encode(cmd, payload=b''):
    size = len(payload)
    body = bytes([size, cmd]) + payload
    crc = 0
    for b in body:
        crc ^= b
    return b'$M<' + body + bytes([crc])

class FCLink:

    def __init__(self, tcp=None, serial=None, baud=115200, rc_hz=100, sim_rc_udp=None, acc_per_g=ACC_PER_G_SITL):
        self.tcp = tcp
        self.serial_path = serial
        self.baud = baud
        self.dev = None
        self.is_serial = serial is not None
        self.sim_rc_udp = sim_rc_udp
        self.rc_sock = None
        self._t0 = 0.0
        self.acc_per_g = acc_per_g
        self.rc = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.rc_dt = 1.0 / rc_hz
        self._rt = None
        self._rxbuf = b''
        self._tlock = threading.Lock()
        self._att = None
        self._imu = None
        self._tt = None

    def connect(self):
        import socket
        if self.is_serial:
            import serial
            self.dev = serial.Serial(self.serial_path, self.baud, timeout=0.02)
        else:
            self.dev = socket.create_connection(self.tcp, 2.0)
            self.dev.settimeout(0.02)
        if self.sim_rc_udp is not None:
            self.rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._t0 = time.time()
        return self

    def _send(self, data):
        if self.is_serial:
            self.dev.write(data)
        else:
            self.dev.sendall(data)

    def _recv(self, n=512):
        try:
            return self.dev.read(n) if self.is_serial else self.dev.recv(n)
        except Exception:
            return b''

    def set_stick(self, roll=None, pitch=None, throttle=None, yaw=None):
        with self.lock:
            if roll is not None:
                self.rc[0] = int(_clip(roll))
            if pitch is not None:
                self.rc[1] = int(_clip(pitch))
            if throttle is not None:
                self.rc[2] = int(_clip(throttle))
            if yaw is not None:
                self.rc[3] = int(_clip(yaw))
        self._send_rc()

    def arm(self, on=True):
        with self.lock:
            self.rc[4] = 2000 if on else 1000
        self._send_rc()

    def _send_rc(self):
        with self.lock:
            ch = list(self.rc)
        if self.sim_rc_udp is not None:
            pkt = struct.pack('<d16H', time.time() - self._t0, *[int(x) for x in ch])
            try:
                self.rc_sock.sendto(pkt, ('127.0.0.1', self.sim_rc_udp))
            except Exception:
                pass
        else:
            self._send(_encode(MSP_SET_RAW_RC, struct.pack('<%dH' % len(ch), *ch)))

    def start_rc_stream(self):

        def loop():
            while not self.stop.is_set():
                self._send_rc()
                time.sleep(self.rc_dt)
        self._rt = threading.Thread(target=loop, daemon=True)
        self._rt.start()

    def _request(self, cmd):
        self._send(_encode(cmd))

    def _read_frame(self, want_cmd, deadline=0.05):
        t0 = time.time()
        while time.time() - t0 < deadline:
            self._rxbuf += self._recv(256)
            i = self._rxbuf.find(b'$M>')
            if i < 0 or len(self._rxbuf) < i + 5:
                continue
            size = self._rxbuf[i + 3]
            cmd = self._rxbuf[i + 4]
            if len(self._rxbuf) < i + 5 + size + 1:
                continue
            payload = self._rxbuf[i + 5:i + 5 + size]
            self._rxbuf = self._rxbuf[i + 6 + size:]
            if cmd == want_cmd:
                return payload
        return None

    def _poll_imu(self, deadline=0.05):
        self._request(MSP_RAW_IMU)
        pl = self._read_frame(MSP_RAW_IMU, deadline)
        if not pl or len(pl) < 12:
            return None
        ax, ay, az, gx, gy, gz = struct.unpack_from('<6h', pl, 0)
        s = G / self.acc_per_g
        acc = [-ax * s, -ay * s, -az * s]
        gyro = [-gx * GYRO_SCALE_DPS, -gy * GYRO_SCALE_DPS, -gz * GYRO_SCALE_DPS]
        return (acc, gyro)

    def _poll_attitude(self, deadline=0.05):
        self._request(MSP_ATTITUDE)
        pl = self._read_frame(MSP_ATTITUDE, deadline)
        if not pl or len(pl) < 6:
            return None
        r, p, y = struct.unpack_from('<3h', pl, 0)
        return (r / 10.0, p / 10.0, float(y))

    def start_telemetry(self, hz=120.0):
        period = 1.0 / hz

        def loop():
            while not self.stop.is_set():
                t0 = time.time()
                a = self._poll_attitude(deadline=0.012)
                if a is not None:
                    with self._tlock:
                        self._att = a
                m = self._poll_imu(deadline=0.012)
                if m is not None:
                    with self._tlock:
                        self._imu = m
                slp = period - (time.time() - t0)
                if slp > 0:
                    time.sleep(slp)
        self._tt = threading.Thread(target=loop, daemon=True)
        self._tt.start()

    def read_attitude(self):
        if self._tt is not None:
            with self._tlock:
                return self._att
        return self._poll_attitude()

    def read_imu(self):
        if self._tt is not None:
            with self._tlock:
                return self._imu
        return self._poll_imu()

    def close(self):
        self.stop.set()
        time.sleep(0.05)
        try:
            self.dev.close()
        except Exception:
            pass

def _clip(x, lo=1000, hi=2000):
    return max(lo, min(hi, int(x)))
