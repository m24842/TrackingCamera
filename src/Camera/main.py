import os
import time
import pigpio
import numpy as np
from threading import Thread
from serial import Serial
from config import FRAME_SHAPE, MOVEMENT_ORDER, MOTOR_V_MAX, MOTOR_A_MAX, BLUETOOTH_PORT, BLUETOOTH_MAC_ADDRESS
from servo import Servo
from camera import TrackingCamera

os.system(f"sudo rfcomm bind /dev/rfcomm0 {BLUETOOTH_MAC_ADDRESS}")

# Status light (ON when camera state is START, OFF when camera state is STOP)
rpi = pigpio.pi()
status_light_pin = 2
rpi.set_mode(status_light_pin, pigpio.OUTPUT)
rpi.write(status_light_pin, 0)

# Bluetooth serial connection to camera remote
remote_bt = Serial(BLUETOOTH_PORT, baudrate=9600, timeout=1)
remote_bt.reset_input_buffer()

# Tracking camera
cap = TrackingCamera(FRAME_SHAPE)

# Servos for 2-axis camera orientation
motor_x, motor_y = Servo(12), Servo(13)

def video_thread():
    """
    Continuously outputs frames from camera to frame buffer / accelerator.
    """
    while True:
        try:
            cap.output_frame()
        except Exception as e:
            print(f"Video thread error: {e}")

def reconnect_remote():
    """
    Reconnects to remote bluetooth device if connection is lost.
    """
    global remote_bt
    try:
        os.system(f"sudo rfcomm bind /dev/rfcomm0 {BLUETOOTH_MAC_ADDRESS}")
        remote_bt.close()
        remote_bt = Serial(BLUETOOTH_PORT, baudrate=9600, timeout=1)
        remote_bt.reset_input_buffer()
    except:
        pass

def recieve_data():
    """
    Recieves data from camera remote.
    """
    data = None
    try:
        if remote_bt.in_waiting > 0:
            data = remote_bt.readline().decode().strip()
            print(data)
    except Exception as e:
        print(f"Remote thread error: {e}")
        reconnect_remote()
    return data

def remote_thread():
    """
    Continuously recieves commands from camera remote.
    START: Start presenter tracking
    STOP: Stop presenter tracking
    CHANGE: Increment target ID
    """
    while True:
        command = recieve_data()
        if command == "START":
            cap.state = "START"
            rpi.write(status_light_pin, 1)
        elif command == "STOP":
            cap.state = "STOP"
            rpi.write(status_light_pin, 0)
        elif command == "CHANGE":
            cap.roll_target_id()
        
        time.sleep(0.1)

def move_fn(x):
    """
    Maps motor movement to servo angle.
    """
    return 180 * np.sign(x) * np.abs(x**MOVEMENT_ORDER)

def motor_thread():
    """
    Locks camera orientation to presenter when tracking is enabled.
    Focuses on hands if lifted otherwise head.
    """
    x_v, x_a = 0, 0
    y_v, y_a = 0, 0
    while True:
        if cap.state == "START":
            cap_focus_x, cap_focus_y = cap.get_focus()
            discrepancy_x = cap_focus_x - 0.5
            discrepancy_y = cap_focus_y - 0.5
            x_a = min(MOTOR_A_MAX, max(-MOTOR_A_MAX, discrepancy_x - x_v))
            x_v = min(MOTOR_V_MAX, max(-MOTOR_V_MAX, x_v + x_a))
            y_a = min(MOTOR_A_MAX, max(-MOTOR_A_MAX, discrepancy_y - y_v))
            y_v = min(MOTOR_V_MAX, max(-MOTOR_V_MAX, y_v + y_a))
            motor_x.move(move_fn(x_v))
            motor_y.move(move_fn(y_v))
            print(cap.target_id, cap_focus_x, cap_focus_y, discrepancy_x, discrepancy_y, motor_x.angle, motor_y.angle, end="\r")
        
        time.sleep(0.01)

if __name__ == "__main__":
    try:
        video = Thread(target=video_thread)
        remote = Thread(target=remote_thread)
        motor = Thread(target=motor_thread)
        
        video.start()
        remote.start()
        motor.start()
        
        video.join()
        remote.join()
        motor.join()
    finally:
        cap.stop()
        remote_bt.close()
        motor_x.cleanup()
        motor_y.cleanup()