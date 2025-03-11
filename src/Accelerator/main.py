import cv2
import time
import numpy as np
from threading import Thread
from serial import Serial
from nanotrack import NanoTrack, YOLOv8Detector

Y_BIAS = 0.1 # Focus on upper part of objects
LASER_THRESHOLD = 255

class Accelerator():
    """
    Accelerator for object tracking and laser detection.
    """
    def __init__(self, output_port, resolution=(1920, 1080), fps=60):
        self.resolution = resolution
        self.fps = fps
        
        # State initialization
        self.objects = {}
        self.laser_position = (-1, -1)
        
        # Object detection and tracking modules
        self.detector = YOLOv8Detector(model_path="src/Accelerator/yolov8n.pt", device='mps')
        self.tracker = NanoTrack()
        
        # Camera source
        self.source = cv2.VideoCapture(1) # OBS Virtual Camera
        
        # Output connection back to tracking camera
        self.output_port = output_port
        self.output = Serial(output_port, baudrate=9600, timeout=1)
        self.output.reset_output_buffer()
        
        self.detection_thread = Thread(target=self.detect)
        self.data_thread = Thread(target=self.send_data)
        
        # self.detection_thread.start()
        self.data_thread.start()
    
    def detect_objects(self):
        """
        Detects and updates object positions.
        """
        ret, frame = self.source.read()
        if not ret: return
        detections = self.detector.detect(frame)
        tracks = self.tracker.update(detections)
        if len(tracks):
            self.objects = {}
            # Get IDs and biased positions of objects
            for track in tracks:
                x1, y1, x2, y2, _, track_id = track[:6]
                self.objects[int(track_id)] = ((x1 + x2) / 2) / self.resolution[0], min(1, Y_BIAS + 1 - ((y1 + y2) / 2) / self.resolution[1])
    
    def detect_laser(self):
        """
        Detects and updates laser position.
        """
        ret, frame = self.source.read()
        if not ret: return
        
        # Laser detection using thresholding
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 250, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 250, 100])
        upper_red2 = np.array([179, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        filtered_frame = mask
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(filtered_frame)
        
        # print(max_val)
        # cv2.imshow("", mask)
        # cv2.waitKey(1)
        
        if max_val >= LASER_THRESHOLD: self.laser_position = (max_loc[0]) / self.resolution[0], 1 - (max_loc[1]) / self.resolution[1]
        else: self.laser_position = (-1, -1)
    
    def detect(self):
        """
        Continuously detects objects and laser.
        """
        while True:
            self.detect_objects()
            self.detect_laser()
            time.sleep(1 / self.fps)
    
    def reconnect(self):
        """
        Reconnects to output port if connection is lost.
        """
        try:
            self.output.close()
            self.output = Serial(self.output_port, baudrate=9600, timeout=1)
            self.output.reset_output_buffer()
        except:
            pass
    
    def send_data(self):
        """
        Continuously sends object and laser positions to tracking camera.
        """
        while True:
            try:
                # Send laser position
                laser_x, laser_y = self.laser_position
                self.output.write(f"LASER {laser_x} {laser_y}\n".encode())
                print(f"LASER {laser_x} {laser_y}", end="\r")

                # Send object ids and positions
                for obj_id, (obj_x, obj_y) in self.objects.items():
                    self.output.write(f"OBJECT {obj_id} {obj_x} {obj_y}\n".encode())
                
                self.output.write(b"END\n")
            except Exception as e:
                print(f"Error: {e}")
                self.reconnect()
            time.sleep(1 / self.fps)
    
    def join(self):
        """
        Joins threads.
        """
        self.detection_thread.join()
        self.data_thread.join()

if __name__ == "__main__":
    try:
        accelerator = Accelerator(output_port="/dev/tty.usbmodem3101", resolution=(1920, 1080), fps=60)
        accelerator.detect()
        accelerator.join()
    finally:
        accelerator.source.release()
        accelerator.output.close()
        cv2.destroyAllWindows()