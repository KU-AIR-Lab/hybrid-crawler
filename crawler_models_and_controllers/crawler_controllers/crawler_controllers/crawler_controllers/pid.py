class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, u_min=None, u_max=None):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.u_min = u_min; self.u_max = u_max
        self._e_prev = 0.0
        self._i = 0.0
    def reset(self):
        self._e_prev = 0.0
        self._i = 0.0
    def step(self, e, dt):
        if dt <= 0: dt = 1e-6
        self._i += e*dt
        d = (e - self._e_prev)/dt
        self._e_prev = e
        u = self.kp*e + self.ki*self._i + self.kd*d
        if self.u_min is not None: u = max(self.u_min, u)
        if self.u_max is not None: u = min(self.u_max, u)
        return u
