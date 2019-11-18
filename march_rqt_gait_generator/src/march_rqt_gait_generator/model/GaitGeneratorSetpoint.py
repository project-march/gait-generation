from march_shared_classes.gait.Setpoint import Setpoint

class GaitGeneratorSetpoint(Setpoint):
    def invert(self, duration):
        self.time = duration - self.time
        self.velocity = -self.velocity
