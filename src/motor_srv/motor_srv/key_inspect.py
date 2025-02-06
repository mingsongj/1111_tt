import torch

checkpoint_path = "model_800.pt"  # Replace with the actual path to your checkpoint file
checkpoint = torch.load(checkpoint_path, map_location=torch.device('cpu'))

print("Keys in the checkpoint file:")
print(checkpoint.keys())  # Prints all top-level keys in the checkpoint

# If the key "model" exists, inspect its contents
if "model" in checkpoint:
    print("\nKeys in 'model':")
    print(checkpoint["model"].keys())
