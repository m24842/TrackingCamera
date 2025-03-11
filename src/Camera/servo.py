import pigpio

class Servo:
    """
    Servo class for controlling camera orientation.
    """
    def __init__(self, pin):
        self.pin = pin
        self.pi = pigpio.pi()
        self.angle = 90
        self.set_angle(self.angle)

    def set_angle(self, angle):
        """
        Set the angle of the servo.

        Args:
            angle (float): Servo angle in degrees.
        """
        self.angle = max(0, min(angle, 180))
        pulse_width = int(500 + (self.angle / 180) * 2000)
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)

    def move(self, d_angle):
        """
        Move the servo by a relative angle.

        Args:
            d_angle (float): Relative angle in degrees.
        """
        self.set_angle(self.angle + d_angle)

    def cleanup(self):
        """
        Clean up servo resources.
        """
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.pi.stop()