import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np


ROOT = os.path.dirname(__file__)
DATA = os.path.join(ROOT, "data")


def read_csv(filename):
    with open(filename, newline="") as file:
        return list(csv.DictReader(file, skipinitialspace=True))


def values(filename, columns):
    rows = read_csv(filename)
    return {column: np.array([float(row[column]) for row in rows]) for column in columns}


def time_axis(pose):
    stamps = np.array([int(row["stamp"]) for row in read_csv(pose)], dtype=float)
    return (stamps - stamps[0]) / 1e9


def aligned_time(pose, count):
    time = time_axis(pose)
    return np.linspace(time[0], time[-1], count)


def finish(axis, title, xlabel, ylabel):
    axis.set_title(title)
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.35)
    axis.legend()


def save_point_plots(output):
    runs = {
        "P": "in_lab_point_P",
        "PID": "in_lab_point_PID",
    }
    colors = {"P": "tab:blue", "PID": "tab:orange"}
    state_colors = {"x": "tab:blue", "y": "tab:green", "theta": "tab:red"}
    controller_styles = {"P": "-", "PID": "--"}

    pose_data = {}
    error_data = {}
    angular_data = {}
    for label, suffix in runs.items():
        pose = os.path.join(DATA, f"robot_pose_{suffix}.csv")
        linear = os.path.join(DATA, f"linear_{suffix}.csv")
        angular = os.path.join(DATA, f"angular_{suffix}.csv")
        pose_data[label] = values(pose, ["x", "y", "theta"])
        error_data[label] = values(linear, ["e", "e_dot"])
        angular_data[label] = values(angular, ["e", "e_dot"])

    fig = plt.figure(figsize=(14, 9))
    grid = fig.add_gridspec(2, 2)
    axes = [fig.add_subplot(grid[0, 0]), fig.add_subplot(grid[0, 1]), fig.add_subplot(grid[1, :])]
    for label in runs:
        time = aligned_time(os.path.join(DATA, f"robot_pose_{runs[label]}.csv"), len(error_data[label]["e"]))
        axes[0].plot(time, error_data[label]["e"], color=colors[label], label=f"{label}: e_linear")
        axes[0].plot(time, error_data[label]["e_dot"], color=colors[label], linestyle="--", label=f"{label}: e_linear_dot")
        angular_time = aligned_time(os.path.join(DATA, f"robot_pose_{runs[label]}.csv"), len(angular_data[label]["e"]))
        axes[1].plot(angular_time, angular_data[label]["e"], color=colors[label], label=f"{label}: e_theta")
        axes[1].plot(angular_time, angular_data[label]["e_dot"], color=colors[label], linestyle="--", label=f"{label}: e_theta_dot")
    finish(axes[0], "Linear point-controller errors", "Time (s)", "Error (m) / error rate (m/s)")
    finish(axes[1], "Angular point-controller errors", "Time (s)", "Error (rad) / error rate (rad/s)")
    for label in runs:
        time = time_axis(os.path.join(DATA, f"robot_pose_{runs[label]}.csv"))
        for state in ["x", "y", "theta"]:
            axes[2].plot(
                time,
                pose_data[label][state],
                color=state_colors[state],
                linestyle=controller_styles[label],
                label=f"{label}: {state}",
            )
    finish(axes[2], "Point controller states", "Time (s)", "State value")
    fig.tight_layout()
    fig.savefig(os.path.join(output, "point_errors_and_states.png"), dpi=200)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(19, 5.5))
    for label in runs:
        axes[0].plot(pose_data[label]["x"], pose_data[label]["y"], color=colors[label], label=label)
        phase_error = error_data[label]["e"]
        phase_rate = error_data[label]["e_dot"]
        axes[1].plot(phase_error, phase_rate, color=colors[label], label=label)
        angular_error = angular_data[label]["e"]
        angular_time = aligned_time(os.path.join(DATA, f"robot_pose_{runs[label]}.csv"), len(angular_error))
        angular_rate = angular_data[label]["e_dot"]
        axes[2].plot(angular_error, angular_rate, color=colors[label], label=label)
    finish(axes[0], "Point controller path", "x (m)", "y (m)")
    finish(axes[1], "Linear error phase plane", "e_linear (m)", "e_linear_dot (m/s)")
    finish(axes[2], "Angular error phase plane", "e_theta (rad)", "e_theta_dot (rad/s)")
    fig.tight_layout()
    fig.savefig(os.path.join(output, "point_path_and_phase.png"), dpi=200)
    plt.close(fig)

    with open(os.path.join(output, "point_metrics.csv"), "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["controller", "threshold_entry_time_s", "settling_time_s", "rmse", "mean_abs_error_after_settling", "post_settling_error_std", "position_error_sign_changes", "angular_overshoot_percent", "angular_error_sign_changes"])
        for label, suffix in runs.items():
            pose = os.path.join(DATA, f"robot_pose_{suffix}.csv")
            error = error_data[label]["e"]
            time = aligned_time(pose, len(error))
            threshold = 0.02
            inside = np.flatnonzero(np.abs(error) <= threshold)
            entry = time[inside[0]] if len(inside) else time[-1]
            outside = np.flatnonzero(np.abs(error) > threshold)
            settling_index = outside[-1] + 1 if len(outside) and outside[-1] + 1 < len(time) else len(time) - 1
            settling = time[settling_index]
            residual = np.mean(np.abs(error[settling_index:]))
            residual_std = np.std(error[settling_index:])
            sign_changes = np.count_nonzero(np.diff(np.sign(error)) != 0)
            angular_error = angular_data[label]["e"]
            angular_overshoot = max(0.0, -np.min(angular_error)) / abs(angular_error[0]) * 100
            angular_sign_changes = np.count_nonzero(np.diff(np.sign(angular_error)) != 0)
            writer.writerow([label, f"{entry:.4f}", f"{settling:.4f}", f"{np.sqrt(np.mean(error ** 2)):.4f}", f"{residual:.4f}", f"{residual_std:.4f}", sign_changes, f"{angular_overshoot:.2f}", angular_sign_changes])


def save_trajectory_plot(output, source_name, output_name, title):
    fig, axis = plt.subplots(figsize=(7, 5.5))
    filename = os.path.join(DATA, f"robot_pose_in_lab_traj_{source_name}.csv")
    pose = values(filename, ["x", "y"])
    axis.plot(pose["x"], pose["y"], label="logged trajectory")
    x = np.linspace(0, 1.5, 200)
    if source_name == "prab":
        axis.plot(x, x ** 2, "k:", label="desired parabola")
    else:
        axis.plot(x, 2 / (1 + np.exp(-2 * x)) - 1, "k:", label="desired sigmoid")
    finish(axis, title, "x (m)", "y (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(output, f"trajectory_{output_name}.png"), dpi=200)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Generate report plots and point-controller metrics.")
    parser.add_argument("--output", default=os.path.join(ROOT, "report_plots"))
    args = parser.parse_args()
    os.makedirs(args.output, exist_ok=True)
    save_point_plots(args.output)
    save_trajectory_plot(args.output, "prab", "parabola", "Parabola trajectory")
    save_trajectory_plot(args.output, "sigma", "sigmoid", "Sigmoid trajectory")
    print(f"Saved report plots and metrics to {args.output}")


if __name__ == "__main__":
    main()



