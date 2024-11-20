class PID():
    def __init__(self, min_val, max_val, p_k, i_k, d_k, dt):
        self.min_val = min_val
        self.max_val = max_val
        self.p_k = p_k
        self.i_k = i_k
        self.d_k = d_k
        self.dt = dt
        self.prev_err = 0
        self.error = None
        self.integral = 0

    def calculate(self, error):
        self.error = error

        prop = self.error
        self.integral += self.error*self.dt
        diff = (self.error-self.prev_err)/self.dt
        self.prev_err = self.error

        self.value = prop*self.p_k+self.integral*self.i_k+diff*self.d_k
        return self.value