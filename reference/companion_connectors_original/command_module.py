"""command_module.py — the per-tick command producer.

Placeholder: each tick it reads vehicle state and the object bearing and outputs a
stick command. Here it outputs a SAFE IDLE command (throttle low, sticks centred) so
the read + command path can be exercised without the vehicle doing anything.

Replace this file with the operational module when you are ready. Same construction
(fc, imu, vision) and the same per-tick .step(t) entry point — nothing else changes.
"""


class CommandModule:
    def __init__(self, fc, imu, vision, logger=None):
        self.fc = fc
        self.imu = imu
        self.vision = vision
        self.log = logger
        self._last = None

    def step(self, t):
        q, w, acc = self.imu.get_state()
        b = self.vision.bearing(t)
        sticks = dict(roll=1500, pitch=1500, yaw=1500, throttle=1000)   # idle: centred, min throttle
        try:
            self.fc.set_stick(**sticks)
        except Exception:
            pass
        if self.log:
            self.log.log(q=q, w=w, acc=acc,
                         bearing=(b.u_world if b else None),
                         conf=(b.conf if b else None),
                         rng=(b.range_m if b else None),
                         sticks=[sticks["roll"], sticks["pitch"], sticks["yaw"], sticks["throttle"]],
                         mode="idle")
        self._last = t
        return sticks
