import os
def get_device_path(symlink_names= ["arduino", "motor"]):
    # Define the symbolic link names
    #symlink_names = ["ttyUSB_Arduino", "ttyUSB_MotorControl"]
    if isinstance(symlink_names, str):
        symlink_names= [symlink_names]
    # Initialize an empty dictionary to store the device paths
    device_paths = {}
    # Loop through the symbolic link names and get their actual device paths
    for symlink_name in symlink_names:
        symlink_path = f"/dev/{symlink_name}"
        
        # Check if the symbolic link exists
        if os.path.exists(symlink_path):
            # Get the actual device path
            device_path = os.path.realpath(symlink_path)
            
            # Store the mapping in the dictionarycat 
            device_paths[symlink_name] = device_path
        else:
            # Handle the case where the symbolic link doesn't exist
            device_paths[symlink_name] = None

    # Print the mapping of symbolic link names to device paths
    for symlink_name, device_path in device_paths.items():
        print(f"Symbolic Link Name: {symlink_name}")
        print(f"Device Path: {device_path}")
        print("\n")
    return list(device_paths.values())[0]
if __name__ == "__main__":
    get_device_path()