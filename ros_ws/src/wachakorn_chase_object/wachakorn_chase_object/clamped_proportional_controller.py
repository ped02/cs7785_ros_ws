from .controller import Controller


class ClampedProportionalController(Controller):
    def __init__(self, kp, command_min, command_max):
        self.kp = kp
        self.command_min = command_min
        self.command_max = command_max

    def execute(self, error, current_time_sec, previous_time_sec):
        output = max(
            min(self.kp * abs(error), self.command_max), self.command_min
        ) * (error / abs(error))

        return output
