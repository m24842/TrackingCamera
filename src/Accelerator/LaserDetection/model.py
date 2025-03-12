import torch
import torch.nn as nn
import torch.nn.functional as F
from diffusers import UNet2DModel

def reward_fn(output, frame_shape):
    x_shape, y_shape = frame_shape

    # Coordinate grids
    x_arrange = torch.arange(x_shape, device=output.device).reshape(1, 1, x_shape, 1)
    y_arrange = torch.arange(y_shape, device=output.device).reshape(1, 1, 1, y_shape)
    
    # Marginal distributions
    x_softmax = F.softmax(torch.sum(output, dim=3, keepdim=True), dim=2)  # Sum over y-axis
    y_softmax = F.softmax(torch.sum(output, dim=2, keepdim=True), dim=3)  # Sum over x-axis
    
    # Expected position (Mean)
    mean_x = torch.sum(x_arrange * x_softmax, dim=2, keepdim=True)
    mean_y = torch.sum(y_arrange * y_softmax, dim=3, keepdim=True)
    
    # Variance computation
    x_variance = torch.sum((x_arrange - mean_x) ** 2 * x_softmax, dim=2)
    y_variance = torch.sum((y_arrange - mean_y) ** 2 * y_softmax, dim=3)

    # Peak probability (max prob in each dimension)
    x_max_prob = torch.max(x_softmax, dim=2).values
    y_max_prob = torch.max(y_softmax, dim=3).values
    
    # Entropy penalty for spread (Optional)
    x_entropy = -torch.sum(x_softmax * torch.log(x_softmax + 1e-8), dim=2)
    y_entropy = -torch.sum(y_softmax * torch.log(y_softmax + 1e-8), dim=3)

    # Reward: High peak probability - low variance - low entropy
    reward = (x_max_prob + y_max_prob) - (x_variance + y_variance) - (x_entropy + y_entropy)
    
    return reward

class LaserDetectionModel(nn.Module):
    def __init__(self, device=None):
        super().__init__()
        self.device = device if device else torch.device("mps")
        self.unet = UNet2DModel(
            in_channels=3,
            out_channels=1,
            norm_num_groups=8,
            down_block_types=("DownBlock2D", "AttnDownBlock2D", "AttnDownBlock2D"),
            up_block_types=("AttnUpBlock2D", "AttnUpBlock2D", "UpBlock2D"),
            block_out_channels=(8, 16, 32),
            layers_per_block=2,
        )
        self.to(self.device)
        
    def forward(self, x):
        timestep = torch.tensor([0], device=self.device)
        return F.relu(self.unet(x, timestep).sample)