import os
import cv2
import random
import matplotlib.pyplot as plt
import torch
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from model import LaserDetectionModel, reward_fn

device = torch.device("mps")
resolution = (128, 128)
dataset_path = "src/Accelerator/LaserDetection/data"
model_path = "src/Accelerator/LaserDetection/model.pt"
optimizer_path = "src/Accelerator/LaserDetection/optimizer.pt"

def load_video(video_path, resolution=(1920, 1080)):
    cap = cv2.VideoCapture(video_path)
    frames = []
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, resolution)
        frame = torch.tensor(frame, dtype=torch.float32) / 255.0
        frames.append(frame.permute(2, 0, 1))
    cap.release()
    return frames

class SyntheticFrameDataset(Dataset):
    def __init__(self, resolution=(1920, 1080)):
        self.resolution = resolution
        self.data = []
        for _ in range(100):
            frame = 7e-1 * torch.rand(3, *resolution)
            center_x = random.randint(10, resolution[0] - 10)
            center_y = random.randint(10, resolution[1] - 10)
            radius = 2
            frame_np = frame.permute(1, 2, 0).numpy()
            frame_np = (frame_np * 255).astype('uint8').copy()
            cv2.circle(frame_np, (center_x, center_y), radius, (255, 0, 0), -1)
            frame = torch.tensor(frame_np, dtype=torch.float32).permute(2, 0, 1) / 255.0
            self.data.append(frame)
        self.length = len(self.data)
        random.shuffle(self.data)
        self.data = torch.stack(self.data)

    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        return self.data[idx]

class FrameDataset(Dataset):
    def __init__(self, path):
        self.data = []
        for file in os.listdir(path):
            video_path = os.path.join(path, file)
            video_tensor = load_video(video_path)
            self.data.extend(video_tensor)
        self.length = len(self.data)
        self.data = random.shuffle(self.data)
        self.data = torch.stack(self.data)

    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        return self.data[idx]

def train(model, optimizer, dataloader, epochs=10):
    for epoch in range(epochs):
        epoch_loss = 0
        for i, frame in enumerate(dataloader):
            frame = frame.to(device)
            optimizer.zero_grad()
            output = model(frame)
            plt.clf()
            plt.imshow(output[0].cpu().detach().numpy().transpose(2, 1, 0))
            plt.pause(0.1)
            loss = -reward_fn(output, resolution).mean()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            epoch_loss += loss.item()
        
        print(f"Epoch {epoch+1}/{epochs}, Reward: {-epoch_loss/len(dataloader)}")
    
    torch.save(model.state_dict(), model_path)
    torch.save(optimizer.state_dict(), optimizer_path)

# dataset = FrameDataset(dataset_path)
dataset = SyntheticFrameDataset(resolution)
dataloader = DataLoader(dataset, batch_size=4, shuffle=True)

model = LaserDetectionModel(device=device)
optimizer = optim.Adam(model.parameters(), lr=1e-3)

if os.path.exists(model_path):
    try:
        model.load_state_dict(torch.load(model_path))
    except:
        print("Failed to load model from state dict.")

if os.path.exists(optimizer_path):
    try:
        optimizer.load_state_dict(torch.load(optimizer_path))
    except:
        print("Failed to load optimizer from state dict.")

epochs = 100
train(model, optimizer, dataloader, epochs)