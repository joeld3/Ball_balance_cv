""" pid.py: A class for creating and updating a PID control system."""

class PID():

    def __init__(self, setpoint, kp=0, ki=0, kd=0, output_max=None, output_min=None, windup_prot=False):
        """Creates a PID controller object

        Args:
            setpoint (int): The desired value of the plant output
            kp (float): The proportional gain coefficient
            ki (float): The integral gain coefficient
            kd (float): The derivative gain coefficient
            output_max (int): The maximum value of the control output. Values above this will be clipped.
            output_min (int): The minimum value of the control output.  Values below this will be clipped.
            windup_prot (bool): Whether to allow anti-windup measures.  The measure implemented in this class only
                allows the integral to sum when the plant output is in a region close to the setpoint. Outside of this
                region the integral sum is set to 0 and the integral term has no effect on the control output.

        """
        self.setpoint = setpoint
        self.curr_output = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0
        self.curr_error = 0
        self.prev_error = 0
        self.output_max = output_max
        self.output_min = output_min
        self.windup_prot = windup_prot
        self.mid = (output_min + output_max)/2

    def set_output(self, val):
        """Sets the current control output and accounts for minimum and maximum limits (if applicable)

        Args:
            val (int): The value to set the control output to

        """
        if self.output_max and (val > self.output_max):
            self.curr_output = self.output_max
        elif self.output_min and (val < self.output_min):
            self.curr_output = self.output_min
        else:
            self.curr_output = val

    def update(self, new_val, time):
        """Updates the PID control based on a new reading of the plant output

        Args:
            new_val (int): The new value of the plant output
            time (float): The time elapsed since the previous update

        """
        self.prev_error = self.curr_error
        self.curr_error = self.setpoint - new_val

        # Proportional Output
        sp = self.curr_error * self.kp

        # Derivative Output
        sd = ((self.curr_error - self.prev_error) * self.kd / time)

        # Integral Output
        if self.windup_prot:
            if abs(self.curr_error) < 100:
                self.error_sum += self.curr_error
                si = (self.error_sum * self.ki)
            else:
                self.error_sum = 0
                si = 0
        else:
            self.error_sum += self.curr_error
            si = (self.error_sum * self.ki)

        pid_total = sp + si + sd
        self.curr_output = self.mid - pid_total
        self.set_output(self.curr_output)