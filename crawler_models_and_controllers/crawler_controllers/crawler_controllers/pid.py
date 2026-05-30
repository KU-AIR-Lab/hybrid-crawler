class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, u_min=None, u_max=None, integral_limit=None):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.u_min = u_min
        self.u_max = u_max
        self.integral_limit = integral_limit
        self._e_prev = 0.0
        self._i = 0.0

    def reset(self):
        self._e_prev = 0.0
        self._i = 0.0

    def step(self, e, dt):
        dt = max(float(dt), 1e-6)
        e = float(e)
        self._i += e * dt
        if self.integral_limit is not None:
            lim = abs(float(self.integral_limit))
            self._i = max(-lim, min(lim, self._i))
        d = (e - self._e_prev) / dt
        self._e_prev = e
        u = self.kp * e + self.ki * self._i + self.kd * d
        if self.u_min is not None:
            u = max(float(self.u_min), u)
        if self.u_max is not None:
            u = min(float(self.u_max), u)
        return float(u)
