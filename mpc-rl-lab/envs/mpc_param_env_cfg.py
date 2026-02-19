import torch

action_space = 5  # [ΔQ_x, ΔQ_y, ΔQ_θ, ΔR_steering, Δhorizon_scale]

# Clip to safe ranges to keep MPC stable
ACTION_SCALE = torch.tensor([2.0, 2.0, 1.0, 0.5, 0.3])