import numpy as np
from serial import Serial
from picamera2 import Picamera2
from threading import Thread
from config import ACCELERATOR_PORT, FRAME_BUFFER_PORT

# Frame buffer out to HDMI
frame_buffer = open(FRAME_BUFFER_PORT, 'wb')

def frame_buffer_write(frame):
    """
    Write camera frame to frame buffer. Sent over HDMI to accelerator.

    Args:
        frame (np.ndarray(int8)): The frame to write to the frame buffer.
    """
    frame = np.flipud(frame) # Camera is upside down
    frame = ((frame[:, :, 0].astype(np.uint16) >> 3) << 11) | ((frame[:, :, 1].astype(np.uint16) >> 2) << 5) | (frame[:, :, 2].astype(np.uint16) >> 3)
    frame = frame.flatten().tobytes()

    frame_buffer.write(frame)
    frame_buffer.flush()
    frame_buffer.seek(0)

class TrackingCamera:
    """
    Camera for tracking objects and laser point.
    """
    def __init__(self, frame_shape, fps=60):
        self.frame_shape = frame_shape
        self.fps = fps
        
        # Camera configuration
        self.camera = Picamera2()
        self.camera.preview_configuration.main.size = frame_shape
        self.camera.preview_configuration.main.format = "BGR888"
        self.camera.configure("preview")
        self.camera.start()
        
        # State intialization
        self.state = "STOP"
        self.target_id = 0
        self.objects = {}
        self.target_position = (0.5, 0.5)
        self.laser_detected = False
        self.laser_position = (-1, -1)
        
        # External accelerator
        self.accelerator = Serial(ACCELERATOR_PORT, baudrate=9600, timeout=1)
        self.accelerator.reset_input_buffer()
        self.accelerator_thread = Thread(target=self.recieve_detections)
        self.accelerator_thread.start()
        
    def reconnect(self):
        """
        Reconnects to accelerator if connection is lost.
        """
        try:
            self.accelerator.close()
            self.accelerator = Serial(ACCELERATOR_PORT, baudrate=9600, timeout=1)
            self.accelerator.reset_input_buffer()
        except:
            pass
        
    def recieve_detections(self):
        """
        Recieves object and laser detections from accelerator.
        """
        while True:
            try:
                if self.accelerator.in_waiting > 0:
                    data = self.accelerator.read_until(b"END\n")
                    if data:
                        # Update object and laser detections
                        lines = data.decode(errors='ignore').splitlines()
                        self.objects = {}
                        for line in lines:
                            split = line.split()
                            if len(split) == 3 and split[0] == "LASER":
                                _, x, y = split
                                self.laser_detected = x != "-1" and y != "-1"
                                self.laser_position = (float(x), float(y))
                            elif len(split) == 4 and split[0] == "OBJECT":
                                _, obj_id, x, y = split
                                self.objects[int(obj_id)] = (float(x), float(y))
                                
                        # Update target position
                        if self.target_id in self.objects: self.target_position = self.objects[self.target_id]
                        
                        # Reset buffer if too much data
                        if self.accelerator.in_waiting > 1000: self.accelerator.reset_input_buffer()
            except Exception as e:
                print(f"Camera thread error: {e}")
                self.reconnect()
    
    def output_frame(self):
        """
        Capture and output a frame to the frame buffer.
        """
        frame = self.camera.capture_array()
        frame_buffer_write(frame)
    
    def roll_target_id(self):
        """
        Increment target object ID if possible, otherwise set to first object ID.
        """
        try:
            sorted_ids = sorted(self.objects.keys())
            if self.target_id in self.objects:
                self.target_id = sorted_ids[(sorted_ids.index(self.target_id) + 1) % len(sorted_ids)]
            else:
                self.target_id = sorted_ids[0]
            self.target_position = self.objects[self.target_id]
        except Exception as e:
            print(f"Camera thread error: {e}")
    
    def stop(self):
        """
        Cleanup camera and accelerator.
        """
        self.camera.stop()
        self.accelerator.close()
        frame_buffer.close()