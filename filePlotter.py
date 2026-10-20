import matplotlib.pyplot as plt
from utilities import FileReader
import argparse
import math

def plot_errors(filename):
    headers, values = FileReader(filename).read_file() 
    time_list = []
    
    if not values:
        print(f"No data found in {filename}")
        return

    # Grab the timestamp index
    stamp_idx = len(headers) - 1
    first_stamp = values[0][stamp_idx]
    
    # Normalize time list to start at t=0 seconds
    for val in values:
        time_list.append((val[stamp_idx] - first_stamp) / 1e9)

    plt.figure(figsize=(10, 6))
    
    for i in range(0, stamp_idx):
        if not isinstance(values[0][i], float):
            continue

        header_name = headers[i]

        if "acc" in header_name.lower():
            label = header_name + " (m/s^2)"

        elif "ang" in header_name.lower():
           label = header_name + " (rad/s)"

        elif header_name.lower() in ["x", "y"]:
            label = header_name +  " (m)"

        elif "th" in header_name.lower():
            label = header_name + " (rad)"

        else:
            label = header_name

        plt.plot(time_list, [row[i] for row in values], label=label)
    
    file_lower = filename.lower()
    if "imu" in file_lower:
        title_str = f"IMU Sensor Measurements - {filename}"
    elif "odom" in file_lower:
        title_str = f"Odometry Measurements - {filename}"
    else:
        title_str = f"Sensor Data Plot - {filename}"

    plt.title(title_str)
    plt.xlabel("Time (seconds)")
    plt.ylabel("Measurements")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_laser(filename):
    headers, values = FileReader(filename).read_file()
    if not values:
        print(f"No data found in {filename}")
        return

    # The manual asks to plot ONLY ONE row. We will grab the very last row 
    # to see what the room looked like at the end of the motion.
    row = values[-1]
    ranges_str = row[0]
    angle_increment = row[1]
    
    # Split the space-separated string back into an array
    ranges = ranges_str.split(' ')
    
    x_points = []
    y_points = []
    
    # Convert polar to Cartesian
    for i, r_str in enumerate(ranges):
        # clean the NaN/Inf data
        if r_str.lower() in ['inf', 'nan', '-inf', '']:
            continue
            
        r = float(r_str)
        
        # filter sensor readings with radius 0
        if r <= 0.0:
            continue
            
        # calculate the angle for this specific laser beam
        theta = i * angle_increment
        
        # convert to Cartesian pose data
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        
        x_points.append(x)
        y_points.append(y)

    # Plot the Cartesian data
    plt.figure(figsize=(8, 8))
    plt.scatter(x_points, y_points, s=10, c='red', marker='o')
    plt.title(f"Cartesian Lidar Map (1 Scan) - {filename}")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.axis('equal') # Forces the X and Y grid to be 1:1 ratio so the room doesn't look squished
    plt.show()

def plot_trajectory(filename):
    headers, values = FileReader(filename).read_file()
    if not values:
        return

    # Find the indices for 'x' and 'y' headers
    x_idx, y_idx = -1, -1
    for i, h in enumerate(headers):
        if h.lower() == 'x':
            x_idx = i
        elif h.lower() == 'y':
            y_idx = i

    if x_idx == -1 or y_idx == -1:
        print(f"Could not find 'x' and 'y' columns in {filename} for trajectory plot.")
        return

    x_vals = [row[x_idx] for row in values]
    y_vals = [row[y_idx] for row in values]

    # Plot X vs Y path
    plt.figure(figsize=(8, 8))
    plt.plot(x_vals, y_vals, label='Robot Path', color='blue', linewidth=1.5)
    
    # Mark start and end points for clarity
    plt.scatter(x_vals[0], y_vals[0], color='green', s=60, label='Start')
    plt.scatter(x_vals[-1], y_vals[-1], color='red', s=60, label='End')

    plt.title(f"Odometry Trajectory (X vs Y) - {filename}")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of CSV files to process')
    
    args = parser.parse_args()
    print("Plotting the files:", args.files)

    for filename in args.files:
        # If the filename has "laser" in it, use our new Cartesian plotter
        if "laser" in filename.lower():
            plot_laser(filename)
        elif "odom" in filename.lower():
            plot_errors(filename)
            plot_trajectory(filename)
        # Otherwise, use the standard line graph plotter for IMU
        else:
            plot_errors(filename)