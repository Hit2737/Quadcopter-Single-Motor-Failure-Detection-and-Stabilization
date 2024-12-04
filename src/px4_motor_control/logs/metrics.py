import pandas as pd
import matplotlib.pyplot as plt
import os


def calculate_stability_metrics(attitude_error, position_error):
    metrics = {}
    metrics["attitude_error_mean"] = attitude_error.mean()
    metrics["attitude_error_std"] = attitude_error.std()
    metrics["position_error_mean"] = position_error.mean()
    metrics["position_error_std"] = position_error.std()
    return metrics


def plot_errors(attitude_error, position_error, motor_id):
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.plot(attitude_error, label=f"Attitude Error Motor {motor_id}")
    plt.xlabel("Time")
    plt.ylabel("Error")
    plt.title(f"Attitude Error Over Time (Motor {motor_id})")
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(position_error, label=f"Position Error Motor {motor_id}")
    plt.xlabel("Time")
    plt.ylabel("Error")
    plt.title(f"Position Error Over Time (Motor {motor_id})")
    plt.legend()

    plt.tight_layout()
    plt.draw()


def main():
    base_path = os.path.dirname(os.path.realpath(__file__))
    print(base_path)
    attitude_error_files = [
        os.path.join(base_path, "attitude_error_NC_1.csv"),
        os.path.join(base_path, "attitude_error_NC_2.csv"),
        os.path.join(base_path, "attitude_error_NC_3.csv"),
        os.path.join(base_path, "attitude_error_NC_4.csv"),
    ]
    position_error_files = [
        os.path.join(base_path, "position_error_NC_1.csv"),
        os.path.join(base_path, "position_error_NC_2.csv"),
        os.path.join(base_path, "position_error_NC_3.csv"),
        os.path.join(base_path, "position_error_NC_4.csv"),
    ]
    print(attitude_error_files)

    overall_attitude_error = pd.DataFrame()
    overall_position_error = pd.DataFrame()

    for i in range(4):
        attitude_error = pd.read_csv(attitude_error_files[i])
        position_error = pd.read_csv(position_error_files[i])

        metrics = calculate_stability_metrics(attitude_error, position_error)
        print(f"Stability Metrics for Motor {i+1}:")
        for key, value in metrics.items():
            print(f"{key}: {value}")

        plot_errors(attitude_error, position_error, i + 1)

        overall_attitude_error = pd.concat(
            [overall_attitude_error, attitude_error], axis=1
        )
        overall_position_error = pd.concat(
            [overall_position_error, position_error], axis=1
        )

    overall_metrics = calculate_stability_metrics(
        overall_attitude_error.mean(axis=2), overall_position_error.mean(axis=2)
    )
    print("Overall Stability Metrics:")
    for key, value in overall_metrics.items():
        print(f"{key}: {value}")

    plot_errors(
        overall_attitude_error.mean(axis=1),
        overall_position_error.mean(axis=1),
        "Overall",
    )
    plt.show()


if __name__ == "__main__":
    main()
