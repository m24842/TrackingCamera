import time
import pigpio
from threading import Thread
from config import MOTOR_RESOLUTION

class Servo:
    """
    Servo class for controlling camera orientation.
    """
    def __init__(self, pin, motion_range=(0, 180)):
        self.pin = pin
        self.pi = pigpio.pi()
        self.angle = 90
        self.target_angle = 90
        self.motion_range = motion_range
        self.running = True
        self.set_angle(self.angle)
        self.motion_thread = Thread(target=self.motion_thread)

    def motion_thread(self):
        """
        Smoothly adjusts the servo angle towards the target angle.
        """
        while self.running:
            if self.angle != self.target_angle and self.angle > self.motion_range[0] and self.angle < self.motion_range[1]:
                dir = 1 if self.target_angle > self.angle else -1
                self.angle += min(dir * MOTOR_RESOLUTION, self.target_angle - self.angle)
                self.set_angle(self.angle)
            
            time.sleep(1e-3)
    
    def set_angle(self, angle):
        """
        Set the angle of the servo.

        Args:
            angle (float): Servo angle in degrees.
        """
        self.angle = max(self.motion_range[0], min(angle, self.motion_range[1]))
        pulse_width = int(500 + (self.angle / 180) * 2000)
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)

    def move(self, d_angle):
        """
        Change the servo target angle by a relative angle.

        Args:
            d_angle (float): Relative angle in degrees.
        """
        self.target_angle += d_angle

    def cleanup(self):
        """
        Clean up servo resources.
        """
        self.running = False
        self.motion_thread.join()
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.pi.stop()