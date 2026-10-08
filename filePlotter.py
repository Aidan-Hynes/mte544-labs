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
            
        plt.plot(time_list, [row[i] for row in values], label=headers[i])
    
    plt.title(f"Data Plot for {filename}")
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
        # 1. Clean the NaN/Inf data
        if r_str.lower() in ['inf', 'nan', '-inf', '']:
            continue
            
        r = float(r_str)
        
        # Sometimes sensors return 0.0 for errors, filter those out too
        if r <= 0.0:
            continue
            
        # Calculate the angle for this specific laser beam
        theta = i * angle_increment
        
        # 2. Convert to Cartesian pose data
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

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of CSV files to process')
    
    args = parser.parse_args()
    print("Plotting the files:", args.files)

    for filename in args.files:
        # If the filename has "laser" in it, use our new Cartesian plotter
        if "laser" in filename.lower():
            plot_laser(filename)
        # Otherwise, use the standard line graph plotter for IMU and Odom
        else:
            plot_errors(filename)