import numpy as np
from serial import Serial
from picamera2 import Picamera2
from threading import Thread
from config import ACCELERATOR_PORT, FRAME_BUFFER_PORT, HAND_SHOULDER_HIP_ANGLE

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
    Camera for tracking presenter.
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
        self.objects = {}
        self.target_id = 1
        
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
        Recieves detections from accelerator.
        """
        while True:
            try:
                if self.accelerator.in_waiting > 0:
                    data = self.accelerator.read_until(b"END\n")
                    if data:
                        self.objects = {}
                        lines = data.decode(errors='ignore').splitlines()
                        for line in lines:
                            split = line.split()
                            # Format:
                            #   ID <id>
                            #   HEAD <x> <y>
                            #   LEFT_SHOULDER <x> <y>
                            #   RIGHT_SHOULDER <x> <y>
                            #   LEFT_HAND <x> <y>
                            #   RIGHT_HAND <x> <y>
                            #   LEFT_HIP <x> <y>
                            #   RIGHT_HIP <x> <y>
                            if len(split) == 23:
                                id = int(split[1])
                                head = (float(split[3]), float(split[4]))
                                left_shoulder = (float(split[6]), float(split[7]))
                                right_shoulder = (float(split[9]), float(split[10]))
                                left_hand = (float(split[12]), float(split[13]))
                                right_hand = (float(split[15]), float(split[16]))
                                left_hip = (float(split[18]), float(split[19]))
                                right_hip = (float(split[21]), float(split[22]))

                                self.objects[id] = {
                                    "head": head,
                                    "left_shoulder": left_shoulder,
                                    "right_shoulder": right_shoulder,
                                    "left_hand": left_hand,
                                    "right_hand": right_hand,
                                    "left_hip": left_hip,
                                    "right_hip": right_hip
                                }
                        
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
        
    def get_focus(self):
        """
        Get the location of focus depending on the presenter's pose.
        """
        # Make sure target exists
        if self.target_id not in self.objects:
            sorted_ids = sorted(self.objects.keys())
            self.target_id = sorted_ids[0] if len(sorted_ids) > 0 else 1
        
        target = self.objects.get(self.target_id)
        
        if target is None: return (0.5, 0.5)
        
        left_shoulder = target["left_shoulder"]
        left_hand = target["left_hand"]
        left_hip = target["left_hip"]
        right_shoulder = target["right_shoulder"]
        right_hand = target["right_hand"]
        right_hip = target["right_hip"]

        def calc_angle(a, b, c):
            ba = np.array(a) - np.array(b)
            bc = np.array(c) - np.array(b)
            norm_ba = np.linalg.norm(ba)
            norm_bc = np.linalg.norm(bc)
            if norm_ba == 0 or norm_bc == 0:
                return 0
            cos_angle = np.dot(ba, bc) / (norm_ba * norm_bc)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Ensure value is within valid range
            angle = np.arccos(cos_angle)
            return np.degrees(angle)

        left_angle = calc_angle(left_hand, left_shoulder, left_hip)

        right_angle = calc_angle(right_hand, right_shoulder, right_hip)
        
        if left_angle > HAND_SHOULDER_HIP_ANGLE or right_angle > HAND_SHOULDER_HIP_ANGLE:
            focus = left_hand if left_angle > right_angle else right_hand
        else: focus = target["head"]
        
        return focus
    
    def roll_target_id(self):
        """
        Increment target ID.
        """
        sorted_ids = sorted(self.objects.keys())
        if self.target_id not in sorted_ids:
            self.target_id = sorted_ids[0] if len(sorted_ids) > 0 else 1
        else:
            self.target_id = sorted_ids[(sorted_ids.index(self.target_id) + 1) % len(sorted_ids)]
    
    def stop(self):
        """
        Cleanup camera and accelerator.
        """
        self.camera.stop()
        self.accelerator.close()
        frame_buffer.close()