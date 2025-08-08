from os import path
import argparse
import json
import logging
import yaml

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np

from pathlib import Path

if __name__ == "__main__":
    # Log settings
    log = logging.getLogger(__name__)
    parser = argparse.ArgumentParser(description="something")
    parser.add_argument("-v", "--verbose", action="count", default=0, dest="verbosity")
    args = parser.parse_args()
    logging.basicConfig()
    logging.getLogger().setLevel(logging.WARN - 10 * args.verbosity)

    # User settings
    package_dir = Path(__file__).parents[1]
    json_dir = f"{package_dir}/../rosbag2json/data/converted_bags"
    if not path.exists(json_dir):
        log.warning(
            f"Directory {json_dir} does not exist! Please ensure that the rosbag2json submodule is cloned"
        )
        exit(1)
    config_dir = f"{package_dir}/config"
    config_path = f"{config_dir}/scripts/plot_interp_data.yaml"
    data_dir = f"{package_dir}/data"
    data_sel_dir = f"{data_dir}/selected_data"

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
    json_name = config["data"]["json_name"]
    t_to_plot = config["data"]["t_to_plot"]
    colors = []
    for color in config["plot_settings"]["colors"]:
        colors.append(mcolors.CSS4_COLORS[color])
    colors = config["plot_settings"]["colors"]
    sizes = config["plot_settings"]["sizes"]
    widths = config["plot_settings"]["widths"]
    n_rows_states = config["plot_settings"]["n_rows_states"]
    n_cols_states = config["plot_settings"]["n_cols_states"]
    plot_x_idx_at_ax_idx = config["plot_settings"]["plot_x_idx_at_ax_idx"]
    x_labels = config["plot_settings"]["x_labels"]

    # Read json data
    json_path = f"{json_dir}/{json_name}.json"
    log.warning(f"Selected json file: {json_path}")
    if not path.exists(json_path):
        raise FileNotFoundError(f"Json path {json_path} not found")
    with open(json_path, "r") as file:
        data = json.load(file)
        data = data["/mpc/rec/interp_data"]
    t = np.array(data["t"])

    # Select and create data to plot
    t = np.array(data["t"])
    idx_to_plot = np.abs(t - t_to_plot).argmin()
    time_to_plot = t[idx_to_plot]
    rk4_states = np.array(data["rk4_states"][idx_to_plot])
    interp_states = np.array(data["interp_states"][idx_to_plot])
    n_rk4_states = rk4_states.shape[0]
    n_interp_states = interp_states.shape[0]
    ts_mpc = t[1] - t[0]
    ts_feedback = ts_mpc / n_interp_states
    t_rk4_states = np.linspace(
        time_to_plot,
        time_to_plot + ts_mpc * (n_rk4_states - 1),
        n_rk4_states,
    )
    t_interp_states = np.linspace(
        time_to_plot,
        time_to_plot + ts_feedback * (n_interp_states - 1),
        n_interp_states,
    )

    # Create plot
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"RK4 and interpolated states at t={time_to_plot:.2f} s",
    )
    fig.suptitle(f"RK4 and interpolated states at t={time_to_plot:.2f} s")
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_x_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        x_idx = plot_x_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].scatter(
            t_rk4_states,
            rk4_states[:, x_idx],
            s=2 * sizes,
            edgecolor=colors[0],
            facecolor="none",
        )
        axes[row_idx, col_idx].scatter(
            t_interp_states,
            interp_states[:, x_idx],
            s=sizes,
            edgecolor="none",
            facecolor=colors[1],
        )
        axes[row_idx, col_idx].set_xlabel(f"Time (s)")
        axes[row_idx, col_idx].set_ylabel(f"{x_labels[x_idx]}")
    plt.show()
