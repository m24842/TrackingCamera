import cv2
import time
import torch
import numpy as np
from serial import Serial
from threading import Thread
from ultralytics import YOLO
from nanotrack import NanoTrack

PORT = "/dev/tty.usbmodem101"

class Accelerator():
    """
    Accelerator for presenter keypoint tracking.
    """
    def __init__(self, output_port, resolution=(1920, 1080), fps=60):
        self.resolution = resolution
        self.fps = fps
        
        # State initialization
        self.objects = {}
        
        # Object detection and tracking modules
        device = "cpu"
        if torch.cuda.is_available():
            device = "cuda"
        elif torch.backends.mps.is_available():
            device = "mps"
        self.detector = YOLO("src/Accelerator/yolo11n-pose.pt").to(device)
        self.tracker = NanoTrack()
        
        # Camera source
        self.source = cv2.VideoCapture(0) # OBS Virtual Camera
        
        # Output connection back to tracking camera
        self.output_port = output_port
        self.output = Serial(output_port, baudrate=9600, timeout=1)
        self.output.reset_output_buffer()
        
        self.detection_thread = Thread(target=self.detect)
        self.data_thread = Thread(target=self.send_data)
        
        # self.detection_thread.start()
        self.data_thread.start()
    
    def detect(self):
        """
        Detects and updates object keypoints.
        """
        while True:
            ret, frame = self.source.read()
            if not ret: continue

            detections = self.detector(frame, stream=True, verbose=False)
            
            # Get keypoints of all detections and save bboxes to update tracker
            track_bboxes = []
            track_poses = []
            sample_frame = frame.copy()
            for detection in detections:
                if len(detection.boxes.xyxy) == 0: continue
                track_bbox = torch.cat((detection.boxes.xyxy[0], detection.boxes.conf, detection.boxes.cls)).cpu().numpy()
                track_bboxes.append(track_bbox)
                
                if detection.keypoints is None: continue
                
                # sample_frame = detection.plot()
                
                object = detection.keypoints.xy[0]
                
                head = object[0].cpu().numpy() / np.array(self.resolution)
                
                left_shoulder = object[5].cpu().numpy() / np.array(self.resolution)
                right_shoulder = object[6].cpu().numpy() / np.array(self.resolution)
                
                left_hand = object[9].cpu().numpy() / np.array(self.resolution)
                right_hand = object[10].cpu().numpy() / np.array(self.resolution)
                
                left_hip = object[11].cpu().numpy() / np.array(self.resolution)
                right_hip = object[12].cpu().numpy() / np.array(self.resolution)
                
                track_poses.append({
                    "head": head,
                    "left_shoulder": left_shoulder,
                    "right_shoulder": right_shoulder,
                    "left_hand": left_hand,
                    "right_hand": right_hand,
                    "left_hip": left_hip,
                    "right_hip": right_hip
                })
            
            self.tracker.update(track_bboxes)
            
            # Match keypoint detections to tracker ids
            self.objects = {}
            for track in self.tracker.tracks:
                track_bbox, track_id = track["bbox"], track["id"]
                idx = next((i for i, bbox in enumerate(track_bboxes) if np.array_equal(bbox, track_bbox)), None)
                if idx is not None:
                    self.objects[track_id] = track_poses[idx]

            # Show a scaled down version of the camera's perspective
            sample_frame = cv2.resize(sample_frame, (self.resolution[0] // 4, self.resolution[1] // 4))
            cv2.imshow("Sample Frame", sample_frame)
            cv2.waitKey(1)
            
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
        Continuously sends presenter keypoints to tracking camera.
        """
        while True:
            try:
                for id, object in self.objects.items():
                    # Format:
                    #   ID <id>
                    #   HEAD <x> <y>
                    #   LEFT_SHOULDER <x> <y>
                    #   RIGHT_SHOULDER <x> <y>
                    #   LEFT_HAND <x> <y>
                    #   RIGHT_HAND <x> <y>
                    #   LEFT_HIP <x> <y>
                    #   RIGHT_HIP <x> <y>
                    self.output.write(f"ID {id} {' '.join([f'{key.upper()} {value[0]} {value[1]}' for key, value in object.items()])}\n".encode())
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
        
accelerator = Accelerator(output_port=PORT, resolution=(1920, 1080), fps=60)

if __name__ == "__main__":
    try:
        accelerator.detect()
        accelerator.join()
    finally:
        accelerator.source.release()
        accelerator.output.close()
        cv2.destroyAllWindows()