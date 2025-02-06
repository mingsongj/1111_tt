import subprocess
import os
import argparse
def update_rules(arduino_vendor_id, arduino_product_id, \
                 motor_vendor_id, motor_product_id):

    # this defines the path to the custom rules file
    rules_file_path = "/etc/udev/rules.d/99-custom-usb.rules"
    # checks to see if the 99-custom-usb.rules file exists, creates an empty file if necessary
    if not os.path.exists(rules_file_path):
        subprocess.run(["sudo", "touch", rules_file_path])
    # reads the content of the custom rules file
    with open(rules_file_path, 'r') as rules_file:
        existing_rules = rules_file.readlines()

    # appends new rules to the updated_rules list if the input device ids do not match the existing rules
    updated_rules = []
    # flags used to determine if a new rule is necessary 
    new_arduino_flag= True
    new_motor_flag= True 
    # checks to see if provided ids match the existing rules 
    for line in existing_rules:
        if f'ATTRS{{idVendor}}=="{arduino_vendor_id}"' in line and f'ATTRS{{idProduct}}=="{arduino_product_id}"' in line:
            new_arduino_flag = False
        if f'ATTRS{{idVendor}}=="{motor_vendor_id}"' in line and f'ATTRS{{idProduct}}=="{motor_product_id}"' in line:
            new_motor_flag = False 
    if new_arduino_flag:
        updated_rules.append(f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{arduino_vendor_id}", ATTRS{{idProduct}}=="{arduino_product_id}", SYMLINK+="arduino"\n')
    if new_motor_flag:
        updated_rules.append(f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{motor_vendor_id}", ATTRS{{idProduct}}=="{motor_product_id}", SYMLINK+="motor"\n')
    # checks to see if new rules need to be added
    if not updated_rules:
        print("Rule update is not necessary!")
    # writes the updated rules to the file
    else:
        with open(rules_file_path, 'a') as rules_file:
            rules_file.writelines(updated_rules)

        # reloads udev rules
        subprocess.run(["sudo", "udevadm", "control", "--reload-rules"])
        # runs udevadm trigger to apply the rules immediately without having to unplug device
        subprocess.run(["sudo", "udevadm", "trigger"])
        print("Udev rules reloaded and trigger executed successfully.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Update rules for USB devices")
    parser.add_argument("--arduino_vendor_id", type=str, default="1a86", help="Arduino Vendor ID")
    parser.add_argument("--arduino_product_id", type=str, default="7523", help="Arduino Product ID")
    parser.add_argument("--motor_vendor_id", type=str, default="0403", help="Motor Vendor ID")
    parser.add_argument("--motor_product_id", type=str, default="6014", help="Motor Product ID")
    args = parser.parse_args()
    # Call the update_rules function with the provided or default arguments
    update_rules(args.arduino_vendor_id, args.arduino_product_id, args.motor_vendor_id, args.motor_product_id)