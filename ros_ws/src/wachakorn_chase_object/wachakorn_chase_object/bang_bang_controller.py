from .controller import Controller


class BangBangController(Controller):
    def __init__(self, deviations, angular_rates):
        self.deviations = deviations
        self.angular_rates = angular_rates

    def execute(self, error, current_time_sec, previous_time_sec):
        for deviation, angular_rate in zip(self.deviations, self.angular_rates):
            if abs(error) < deviation:
                return -(error / abs(error)) * angular_rate

        # Exceeded max bounds
        return 0.0
