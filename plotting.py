"""Basic post-run plots and tracking statistics. Never called in the loop."""

import numpy as np
import matplotlib.pyplot as plt


def compute_statistics(rows):
    """Return one row per X/Y/Z/vector norm, using actual minus VAC target.

    Columns: force peak [N], force RMS [N], position RMSE [mm], position
    MAE [mm], position peak error [mm]. The target includes compliance:
    comparing actual to the nominal reference would also count the intended
    force-induced displacement as a tracking error.
    """
    force = np.asarray([row["force"] for row in rows])
    target = np.asarray([row["target_position"] for row in rows])
    actual = np.asarray([row["actual_position"] for row in rows])
    error_mm = 1000.0 * (actual - target)
    force = np.column_stack((force, np.linalg.norm(force, axis=1)))
    error_mm = np.column_stack((error_mm, np.linalg.norm(error_mm, axis=1)))
    return np.column_stack((
        np.max(np.abs(force), axis=0),
        np.sqrt(np.mean(force ** 2, axis=0)),
        np.sqrt(np.mean(error_mm ** 2, axis=0)),
        np.mean(np.abs(error_mm), axis=0),
        np.max(np.abs(error_mm), axis=0),
    ))


def plot_results(rows, config, folder, show=True):
    """Save two figures and a position-error statistics CSV; optionally display the figures."""
    if not rows:
        return
    t = np.asarray([row["time"] for row in rows])
    colors = ("tab:blue", "tab:orange", "tab:green")
    title = "{} / {} / {}".format(config.environment, config.force_mode, config.example)

    # Rows: XYZ. Columns: position, velocity, acceleration.
    trajectory, axes = plt.subplots(3, 3, figsize=(13, 9), sharex=True)
    quantities = (("position", "Position [m]"), ("velocity", "Velocity [m/s]"),
                  ("acceleration", "Acceleration [m/s^2]"))
    for column, (quantity, label) in enumerate(quantities):
        reference = np.asarray([row["reference_" + quantity] for row in rows])
        desired = np.asarray([row["target_" + quantity] for row in rows])
        measured = np.asarray([row["actual_" + quantity] for row in rows])
        for i, axis_name in enumerate("XYZ"):
            axis = axes[i, column]
            axis.plot(t, reference[:, i], ":", color="gray", label="Nominal reference")
            axis.plot(t, measured[:, i], color="tab:blue", label="Actual")
            axis.plot(t, desired[:, i], "--", color="tab:orange", label="Reference + VAC")
            axis.grid(alpha=0.3)
            if column == 0:
                axis.set_ylabel(axis_name)
            if i == 0:
                axis.set_title(label)
            if i == 2:
                axis.set_xlabel("Time [s]")
    axes[0, 0].legend(fontsize=8)
    trajectory.suptitle("TCP motion | " + title + " | actual acceleration: velocity difference")
    trajectory.tight_layout()
    trajectory.savefig(folder / "trajectory.png", dpi=160)

    signals, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for axis, name, label in zip(axes, ("force", "damping", "stiffness"),
                                  ("VAC input force [N]", "Damping [N s/m]", "Stiffness [N/m]")):
        values = np.asarray([row[name] for row in rows])
        for i, axis_name in enumerate("XYZ"):
            axis.plot(t, values[:, i], ("-", "--", ":")[i], color=colors[i], label=axis_name)
        axis.set_ylabel(label)
        axis.grid(alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel("Time [s]")
    signals.suptitle("Force and applied parameters | " + title)
    signals.tight_layout()
    signals.savefig(folder / "force_parameters.png", dpi=160)

    metrics = compute_statistics(rows)
    names = ("X", "Y", "Z", "norm")
    with (folder / "statistics.csv").open("w", encoding="utf-8") as stream:
        stream.write("axis,force_peak_N,force_rms_N,error_rmse_mm,error_mae_mm,error_peak_mm\n")
        for name, values in zip(names, metrics):
            stream.write(name + "," + ",".join("{:.9g}".format(v) for v in values) + "\n")
    print("\n{} samples; controller time {:.3f} s".format(len(rows), t[-1]))
    print("Axis   Peak force [N]   RMS force [N]   RMSE [mm]   MAE [mm]   Peak error [mm]")
    for name, values in zip(names, metrics):
        print("{:<5} {:>14.4f} {:>15.4f} {:>11.4f} {:>10.4f} {:>17.4f}".format(name, *values))
    if show:
        plt.show()
    else:
        for figure in (trajectory, signals):
            plt.close(figure)
