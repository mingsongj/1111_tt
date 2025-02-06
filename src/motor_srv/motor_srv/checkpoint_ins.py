import torch

checkpoint = torch.load("model_800.pt")
print(checkpoint.keys())  # Check if it contains 'state_dict' or something else
