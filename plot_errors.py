import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))


    odom_x = headers.index("odom_x")
    odom_y = headers.index("odom_y")
    kf_x = headers.index("kf_x")
    kf_y = headers.index("kf_y")

    axes[0, 0].plot([row[odom_x] for row in values],
                    [row[odom_y] for row in values],
                 color="blue", label="Odometry")
    axes[0, 0].plot([row[kf_x] for row in values],
                    [row[kf_y] for row in values],
                 color="orange", label="Kalman filter")
    axes[0, 0].set_title("Odometry and Kalman filter paths")
    axes[0, 0].set_xlabel("x (m)")
    axes[0, 0].set_ylabel("y (m)")
    axes[0, 0].legend()
    axes[0, 0].grid()

    imu_ax = headers.index("imu_ax")
    imu_ay = headers.index("imu_ay")
    kf_ax = headers.index("kf_ax")
    kf_ay = headers.index("kf_ay")
    axes[0, 1].plot(time_list, [row[imu_ax] for row in values], label="IMU ax")
    axes[0, 1].plot(time_list, [row[kf_ax] for row in values], label="KF ax")
    axes[0, 1].plot(time_list, [row[imu_ay] for row in values], label="IMU ay")
    axes[0, 1].plot(time_list, [row[kf_ay] for row in values], label="KF ay")
    axes[0, 1].set_title("Acceleration: measurements and estimates")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_ylabel("acceleration (m/s^2)")
    axes[0, 1].legend()
    axes[0, 1].grid()

    axes[1, 0].plot(time_list, [row[odom_x] for row in values], label="Odometry x")
    axes[1, 0].plot(time_list, [row[kf_x] for row in values], label="KF x")
    axes[1, 0].plot(time_list, [row[odom_y] for row in values], label="Odometry y")
    axes[1, 0].plot(time_list, [row[kf_y] for row in values], label="KF y")
    axes[1, 0].set_title("Position: odometry and estimates")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("position (m)")
    axes[1, 0].legend()
    axes[1, 0].grid()

    kf_vx = headers.index("kf_vx")
    kf_w = headers.index("kf_w")
    axes[1, 1].plot(time_list, [row[kf_vx] for row in values], label="KF linear velocity")
    axes[1, 1].plot(time_list, [row[kf_w] for row in values], label="KF angular velocity")
    axes[1, 1].set_title("Estimated velocities")
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].set_ylabel("velocity")
    axes[1, 1].legend()
    axes[1, 1].grid()

    fig.tight_layout()

    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)


