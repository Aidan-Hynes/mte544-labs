import csv


def _read_pose_log(filename):
    with open(filename, newline="") as file:
        reader = csv.reader(file, skipinitialspace=True)
        if next(reader, None) is None:
            raise ValueError(f"{filename} is empty")

        rows = []
        for line_number, row in enumerate(reader, start=2):
            while row and not row[-1].strip():
                row.pop()
            if not row:
                continue
            if len(row) != 4:
                raise ValueError(
                    f"Expected 4 values in {filename} on line {line_number}, "
                    f"found {len(row)}"
                )
            try:
                rows.append([
                    float(row[0].strip()),
                    float(row[1].strip()),
                    float(row[2].strip()),
                    int(row[3].strip()),
                ])
            except ValueError as error:
                raise ValueError(
                    f"Invalid numeric data in {filename} on line {line_number}"
                ) from error

    if not rows:
        raise ValueError(f"{filename} contains no pose samples")

    x_values = [row[0] for row in rows]
    y_values = [row[1] for row in rows]
    theta_values = [row[2] for row in rows]
    first_stamp_ns = rows[0][3]
    time_values = [(row[3] - first_stamp_ns) * 1e-9 for row in rows]
    return x_values, y_values, theta_values, time_values


def plot_errors(filename="robotPose.csv"):
    import matplotlib.pyplot as plt

    x_values, y_values, theta_values, time_values = _read_pose_log(filename)
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].plot(x_values, y_values)
    axes[0].set_title("Robot trajectory")
    axes[0].set_xlabel("X position (m)")
    axes[0].set_ylabel("Y position (m)")
    axes[0].axis("equal")
    axes[0].grid(True)

    axes[1].plot(time_values, theta_values)
    axes[1].set_title("Heading over time")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Heading (rad)")
    axes[1].grid(True)

    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_errors()
