import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(2,1, figsize=(14,6))


    odom_x = headers.index("odom_x")
    odom_y = headers.index("odom_y")
    kf_x = headers.index("kf_x")
    kf_y = headers.index("kf_y")

    axes[0].plot([row[odom_x] for row in values],
                 [row[odom_y] for row in values],
                 color="blue", label="Odometry")
    axes[0].plot([row[kf_x] for row in values],
                 [row[kf_y] for row in values],
                 color="orange", label="Kalman filter")
    axes[0].set_title("Odometry and Kalman filter paths")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].legend()
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()

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


