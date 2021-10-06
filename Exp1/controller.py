class P_controller:
    def init(self, k):
        self.k = k

    def controll(self, error):
        return self.k * error

class PID_controller:
    pass