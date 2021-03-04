""" servo.py: A class that details a typical hobbyist servo motor."""

class Servo():
    """An object representing the controls of a servo motor."""

    def __init__(self, pwm_steps, inverted = False, p90_val = None, n90_val = None):
        """Creates a servo object

        Args:
            pwm_steps (int): The PWM resolution
            inverted (bool): Flips the standard servo control duty cycles of 5% to 20% to 80% to 95%.
            p90_val (int): The PWM output value that drives the servo to the positive 90 degree position
            n90_val (int): The PWM output value that drives the servo to the negative 90 degree position
        """
        self.pwm_steps = pwm_steps
        self.inverted = inverted
        self.p90_val = p90_val
        self.n90_val = n90_val

        if not self.inverted:
            if not p90_val:
                self.p90_val = round(self.pwm_steps/20)
            if not n90_val:
                self.n90_val = round(self.pwm_steps/10)
        else:
            if not p90_val:
                self.p90_val = round(self.pwm_steps*19/20)
            if not n90_val:
                self.n90_val = round(self.pwm_steps*9/10)

        self.center = (self.p90_val + self.n90_val)/2
        self.m, self.b = self.linearize_angle()

    def linearize_angle(self):
        """ Maps the desired angle of a servo motor to the PWM output that will achieve the angle

        Returns:
            The slope and y-intercept of the line representing the angle/pwm output relationship
        """
        m = (self.p90_val - self.n90_val) / 180
        b = self.p90_val - 90*m
        return m,b

    def angle_to_out(self, set_angle):
        """Takes a desired servo angle and produces the PWM output to achieve the angle

        Returns:
            The PWM output value that will achieve the desired angle
        """
        return round(set_angle*self.m + self.b)

