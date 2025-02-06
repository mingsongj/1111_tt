import sys
import csv
from pathlib import Path
from rosbags.highlevel import AnyReader
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math
import matplotlib.pyplot as plt

distance_measured = True

if len(sys.argv) != 2:
    print("Usage: python read_rosbag.py <bag_file>")
    sys.exit(1)

bag_file = sys.argv[1]
bag_file_path = Path(bag_file)

# Get the directory and filename of the original bag file
bag_dir = bag_file_path.parent
bag_filename = bag_file_path.stem

# Create subdirectories for CSV and graphs within the "processed" folder
processed_dir = bag_dir.parent / 'processed'  # Adjust path to be at the same level as "bags"
dir = processed_dir / bag_filename
dir.mkdir(parents=True, exist_ok=True)  # Create subdirectory for csvs and plots 

# Create a reader instance and open the bag for reading
with AnyReader([bag_file_path]) as reader:
    connections = [x for x in reader.connections if x.topic == '/power' or x.topic == '/visual_slam/tracking/vo_pose']

    # Initialize lists to store power and pose data
    power_data = []
    pose_data = []
    start_time = None

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        if start_time is None:
            start_time = timestamp

        if connection.topic == '/power':
            # Deserialize 'power' message
            msg = reader.deserialize(rawdata, connection.msgtype)
            power_data.append([timestamp - start_time, msg.data])

        elif connection.topic == '/visual_slam/tracking/vo_pose':
            # Deserialize 'vo_pose' message
            msg = reader.deserialize(rawdata, connection.msgtype)
            if len(pose_data) > 0:
                # Calculate Euclidean distance from the first pose in the series
                prev_pose = pose_data[0][1]
                dx = prev_pose.pose.position.x - msg.pose.position.x
                dy = prev_pose.pose.position.y - msg.pose.position.y
                dz = prev_pose.pose.position.z - msg.pose.position.z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                pose_data.append([timestamp - start_time, msg, distance])
            else:
                pose_data.append([timestamp - start_time, msg, 0])

if len(pose_data) == 0:
    distance_measured= False # no distance measured 

# Create separate lists for power, time, and distance
time_power = [entry[0] / 1e9 for entry in power_data]
power = [entry[1] for entry in power_data]
if distance_measured:
    time_distance = [entry[0] / 1e9 for entry in pose_data]
    distance = [entry[2] for entry in pose_data]

# Calculate total energy (integral of power over time)
total_energy = sum([(power[i] * (time_power[i] - time_power[i - 1])) for i in range(1, len(power))])

# Calculate COT (Cost of Transport)
if distance_measured:
    robot_mass = 7 #with shell 7kg. without shell 6.3kg. 
    cot = total_energy / ((distance[-1] - distance[0])*9.8*robot_mass)

# Save the data to CSV files in the CSV subdirectory
csv_file_power = dir / 'power.csv'
with open(csv_file_power, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Timestamp', 'Power Data'])
    for i in range(len(time_power)):
        csvwriter.writerow([time_power[i], power[i]])

if distance_measured:

    csv_file_dist = dir / 'dist.csv'
    with open(csv_file_dist, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Timestamp', 'Distance'])
        for i in range(len(time_distance)):
            csvwriter.writerow([time_distance[i], distance[i]])

# Create a graph for power
plt.figure(1)
plt.plot(time_power, power)
plt.xlabel('Time')
plt.ylabel('Power Data')
plt.title('Power Data Over Time')

if distance_measured:

    # Create a graph for distance
    plt.figure(2)
    plt.plot(time_distance, distance)
    plt.xlabel('Time')
    plt.ylabel('Distance')
    plt.title('Euclidean Distance Over Time')

# Save the graphs in the graphs subdirectory
power_graph = dir / 'power.png'
plt.figure(1)
plt.savefig(power_graph)

if distance_measured:
    distance_graph =  dir /'dist.png'
    plt.figure(2)
    plt.savefig(distance_graph)

# Get the base name of the bag file
bag_base_name = bag_file_path.name

# Check if the "data.csv" file already exists
data_csv = processed_dir / 'data.csv'
header_exists = data_csv.is_file()

# Save the total energy, distance, COT, and bag file name to "data.csv" (only write header if the file didn't exist)
if distance_measured:
    with open(data_csv, 'a', newline='') as datafile:
        csvwriter = csv.writer(datafile)
        if not header_exists:
            csvwriter.writerow(['Bag File Name', 'Total Energy', 'Total Distance', 'COT'])
        csvwriter.writerow([bag_base_name, total_energy, distance[-1] - distance[0], cot])
else:
     with open(data_csv, 'a', newline='') as datafile:
        csvwriter = csv.writer(datafile)
        if not header_exists:
            csvwriter.writerow(['Bag File Name', 'Total Energy', 'Total Distance', 'COT'])
        csvwriter.writerow([bag_base_name, total_energy])

