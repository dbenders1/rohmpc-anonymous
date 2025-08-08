import argparse
import json
import logging
import math
import time
import yaml

import matplotlib.pyplot as plt
import numpy as np

from os import path
from pathlib import Path
from mpc_model_id_mismatch import helpers

if __name__ == "__main__":
    # Start timing
    start = time.time()

    # Log settings
    log = logging.getLogger(__name__)
    parser = argparse.ArgumentParser(description="something")
    parser.add_argument("-v", "--verbose", action="count", default=0, dest="verbosity")
    args = parser.parse_args()
    logging.basicConfig()
    logging.getLogger().setLevel(logging.WARN - 10 * args.verbosity)

    # Default settings
    compute_rho_c = False
    compute_epsilon = False
    compute_w_bar_c = False

    # User settings
    package_dir = Path(__file__).parents[1]
    runtime_json_dir = f"{package_dir}/../mpc/mpc_tools/recorded_data"
    if not path.exists(runtime_json_dir):
        log.warning(
            f"Directory {runtime_json_dir} does not exist! Please ensure that the mpc submodule is cloned"
        )
        exit(1)
    ros_rec_json_dir = f"{package_dir}/../rosbag2json/data/converted_bags"
    if not path.exists(ros_rec_json_dir):
        log.warning(
            f"Directory {ros_rec_json_dir} does not exist! Please ensure that the rosbag2json submodule is cloned"
        )
        exit(1)
    config_dir = f"{package_dir}/config"
    config_path = f"{config_dir}/scripts/determine_tightening.yaml"
    data_dir = f"{package_dir}/data"
    data_sel_dir = f"{data_dir}/selected_data"
    output_data_dir = f"{data_dir}/tightening_results"
    output_data_json_path = f"{output_data_dir}/tightening.json"
    fig_dir = f"{data_dir}/figures/tightening"

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
    g = config["constants"]["g"]

    runtime_json_names = config["data"]["runtime_json_names"]
    ros_rec_json_names = config["data"]["ros_rec_json_names"]
    n_idx_ignore = config["data"]["n_idx_ignore"]
    t_cut_circle = config["data"]["t_cut_circle"]
    t_cut_lemniscate = config["data"]["t_cut_lemniscate"]

    compute_settings = config["compute_settings"]
    compute_x_in_eps_ball = compute_settings["compute_x_in_eps_ball"]
    x_in_eps_ball_nom = np.array(compute_settings["x_in_eps_ball_nom"])
    x_in_eps_ball_dir = np.array(compute_settings["x_in_eps_ball_dir"])

    do_plot = config["do_plot"]
    do_plot_lyap_err = do_plot["lyap_err"]
    do_plot_rpi_tightening_over_rho_c = do_plot["rpi_tightening_over_rho_c"]
    do_plot_w_bar_c_time = do_plot["w_bar_c_time"]
    do_plot_w_bar_c_sorted = do_plot["w_bar_c_sorted"]
    do_plot_epsilon_time = do_plot["epsilon_time"]
    do_plot_epsilon_sorted = do_plot["epsilon_sorted"]

    save_settings = config["save_settings"]
    save_lyap_err = save_settings["lyap_err"]
    if save_lyap_err:
        do_plot_lyap_err = True

    quad_name = config["model"]["name"]

    plot_settings = config["plot_settings"]
    linewidth = plot_settings["linewidth"]
    n_rows_plot = plot_settings["n_rows"]
    n_cols_plot = plot_settings["n_cols"]
    plot_stage_idx_at_ax_idx = plot_settings["plot_stage_idx_at_ax_idx"]

    # Print warning if the number of runtime and ros recording json names do not match
    n_runtime_json_names = len(runtime_json_names)
    n_ros_rec_json_names = len(ros_rec_json_names)
    if n_runtime_json_names != n_ros_rec_json_names:
        raise ValueError(
            f"Number of runtime_json_names ({n_runtime_json_names}) and ros_rec_json_names ({n_ros_rec_json_names}) must match"
        )

    # Create model
    quad_name = config["model"]["name"]
    g = config["constants"]["g"]
    params_file = f"{config_dir}/systems/{quad_name}.yaml"
    if path.exists(params_file):
        log.warning(f"Selected {quad_name} params file")
    else:
        log.fatal(f"Unknown quad name {quad_name}! Exiting")
        exit(1)
    if quad_name == "falcon":
        model = helpers.DroneAgiModel(quad_name, g, params_file)
    else:
        log.fatal(f"Unknown model {quad_name}! Exiting")
        exit(1)

    # Determine what quantities to compute
    first_ros_rec_json_name = ros_rec_json_names[0]
    if "rho_c" in first_ros_rec_json_name:
        compute_rho_c = True
    if "epsilon" in first_ros_rec_json_name:
        compute_epsilon = True
    if "w_bar_c" in first_ros_rec_json_name:
        compute_w_bar_c = True
    if not (compute_rho_c or compute_epsilon or compute_w_bar_c):
        log.fatal(
            f"First ros_rec_json_name {first_ros_rec_json_name} does not contain 'epsilon', 'rho_c', or 'w_bar_c'! Exiting"
        )
        exit(1)
    print(f"compute_rho_c: {compute_rho_c}")
    print(f"compute_epsilon: {compute_epsilon}")
    print(f"compute_w_bar_c: {compute_w_bar_c}")

    use_nom_ref = compute_rho_c
    use_pred_traj = compute_epsilon or compute_w_bar_c

    # When computing w_bar_c:
    # - We can use data from multiple closed-loop MPC trajectories to construct the tube. Therefore, we keep track of all relevant data of the loops below in a dict
    # - Read optimized rho_c from the output_data_json_path
    if compute_w_bar_c:
        files_dict = {}
        with open(output_data_json_path, "r") as file:
            tightening_data = json.load(file)
        if "rho_c" in tightening_data:
            rho_c = tightening_data["rho_c"]
        else:
            log.fatal(
                f"rho_c not found in {output_data_json_path}! Please ensure that the file contains a valid rho_c value"
            )

    # Iterate over all runtime and ros recording json names
    for file_idx in range(n_ros_rec_json_names):
        # Get file names
        runtime_json_name = runtime_json_names[file_idx]
        ros_rec_json_name = ros_rec_json_names[file_idx]

        # Ensure the current file in this loop is consistent in settings with the first file, otherwise exit
        if ("rho_c" in ros_rec_json_name) != compute_rho_c:
            log.fatal(
                f"Current ros_rec_json_name {ros_rec_json_name} does not match compute_rho_c setting {compute_rho_c}! Exiting"
            )
            exit(1)
        if ("epsilon" in ros_rec_json_name) != compute_epsilon:
            log.fatal(
                f"Current ros_rec_json_name {ros_rec_json_name} does not match compute_epsilon setting {compute_epsilon}! Exiting"
            )
            exit(1)
        if ("w_bar_c" in ros_rec_json_name) != compute_w_bar_c:
            log.fatal(
                f"Current ros_rec_json_name {ros_rec_json_name} does not match compute_w_bar_c setting {compute_w_bar_c}! Exiting"
            )
            exit(1)

        if compute_epsilon or compute_w_bar_c:
            files_dict[ros_rec_json_name] = {}

        # Read ROS runtime json data
        runtime_json_path = f"{runtime_json_dir}/{runtime_json_name}.json"
        log.warning(f"Selected runtime json file: {runtime_json_path}")
        if not path.exists(runtime_json_path):
            raise FileNotFoundError(f"Runtime json path {runtime_json_path} not found")
        with open(runtime_json_path, "r") as file:
            runtime_data = json.load(file)

        runtime_static_data = runtime_data["static_data"]
        stepsize_tmpc = runtime_static_data["stepsize"]
        steps_tmpc = runtime_static_data["steps"]
        w_bias = np.array(runtime_static_data["w_bias"])
        P_delta = np.array(runtime_static_data["P_delta"])

        # Read ROS recording json data
        ros_rec_json_path = f"{ros_rec_json_dir}/{ros_rec_json_name}.json"
        log.warning(f"Selected ROS recording json file: {ros_rec_json_path}")
        if not path.exists(ros_rec_json_path):
            raise FileNotFoundError(
                f"ROS recording json path {ros_rec_json_path} not found"
            )
        with open(ros_rec_json_path, "r") as file:
            ros_rec_data = json.load(file)

        time_precision = ros_rec_data["time_precision"]

        data_x_cur = ros_rec_data["/falcon/ground_truth/odometry"]
        t_x_cur = np.array(data_x_cur["t"])
        x_cur = np.array(data_x_cur["x"])

        data_x_cur_est = ros_rec_data["/mpc/rec/current_state"]
        t_x_cur_est = np.array(data_x_cur_est["t"])
        x_cur_est = np.array(data_x_cur_est["current_state"])

        if use_nom_ref:
            data_nom_ref = ros_rec_data["/mpc/rec/nominal_reference"]
            t_nom_ref = np.array(data_nom_ref["t"])
            x_nom_ref = np.array(data_nom_ref["x_ref"])
            u_nom_ref = np.array(data_nom_ref["u_ref"])

            # Cut data when reference input contains a sudden change
            first_u_norm = np.linalg.norm(u_nom_ref[0])
            for t_idx in range(len(t_nom_ref)):
                if abs(np.linalg.norm(u_nom_ref[t_idx]) - first_u_norm) > 2:
                    log.warning(
                        f"Cutting nominal reference data after time {t_nom_ref[t_idx - 1]} because input norm exceeds threshold value"
                    )
                    t_nom_ref = t_nom_ref[:t_idx]
                    x_nom_ref = x_nom_ref[:t_idx]
                    u_nom_ref = u_nom_ref[:t_idx]
                    break

        if use_pred_traj:
            data_pred_traj = ros_rec_data["/mpc/rec/predicted_trajectory/0"]
            t_pred_traj = np.array(data_pred_traj["t"])
            u_pred_traj = np.array(data_pred_traj["u_pred"])
            x_pred_traj = np.array(data_pred_traj["x_pred"])

            # Cut data after a specified amount of time
            if "circle" in ros_rec_json_name:
                t_cut = t_cut_circle
            elif "lemniscate" in ros_rec_json_name:
                t_cut = t_cut_lemniscate
            else:
                t_cut = np.inf
            pred_traj_idc = np.where(t_pred_traj <= t_cut)[0]
            if len(pred_traj_idc) == 0:
                log.warning(
                    f"No predicted trajectory data found before time {t_cut} in {ros_rec_json_name}! Skipping this file."
                )
                continue
            t_pred_traj = t_pred_traj[pred_traj_idc]
            u_pred_traj = u_pred_traj[pred_traj_idc]
            x_pred_traj = x_pred_traj[pred_traj_idc]

        # Set times to a specific precision
        t_x_cur_est = np.round(t_x_cur_est, time_precision)
        if use_nom_ref:
            t_nom_ref = np.round(t_nom_ref, time_precision)
        if use_pred_traj:
            t_pred_traj = np.round(t_pred_traj, time_precision)

        # Determine various parameters of runtime data
        nx = x_cur.shape[1]
        n_tmpc = len(t_x_cur_est)
        if use_nom_ref:
            n_tmpc = min(len(t_x_cur_est), len(t_nom_ref))
        elif use_pred_traj:
            n_tmpc = min(len(t_x_cur_est), len(t_pred_traj))
        dt_tmpc = steps_tmpc * stepsize_tmpc

        # Align all data recorded in mpc
        t_x_cur_est = t_x_cur_est[:n_tmpc]
        x_cur_est = x_cur_est[:n_tmpc]
        x_cur_est = x_cur_est[:n_tmpc]
        if use_nom_ref:
            t_nom_ref = t_nom_ref[:n_tmpc]
            x_nom_ref = x_nom_ref[:n_tmpc]
            u_nom_ref = u_nom_ref[:n_tmpc]
        if use_pred_traj:
            t_pred_traj = t_pred_traj[:n_tmpc]
            u_pred_traj = u_pred_traj[:n_tmpc]
            x_pred_traj = x_pred_traj[:n_tmpc]

        # Ensure that all times are aligned
        if t_x_cur_est[-1] > t_x_cur[-1]:
            log.warning(
                "The estimated state has a recording after the ground truth state. Shrinking t_x_cur_est, x_cur_est, and (t_pred_traj, u_pred_traj, x_pred_traj) or (t_nom_ref, u_nom_ref, x_nom_ref) to the last time of t_x_cur"
            )
            t_x_cur_est = t_x_cur_est[t_x_cur_est <= t_x_cur[-1]]
            x_cur_est = x_cur_est[: len(t_x_cur_est)]
            if compute_rho_c:
                t_nom_ref = t_nom_ref[t_nom_ref <= t_x_cur[-1]]
                u_nom_ref = u_nom_ref[: len(t_nom_ref)]
                x_nom_ref = x_nom_ref[: len(t_nom_ref)]
            if use_pred_traj:
                t_pred_traj = t_pred_traj[t_pred_traj <= t_x_cur[-1]]
                u_pred_traj = u_pred_traj[: len(t_pred_traj)]
                x_pred_traj = x_pred_traj[: len(t_pred_traj)]
        print(f"t start: {t_x_cur_est[0]}")
        print(f"t end: {t_x_cur_est[-1]}")

        if compute_rho_c or compute_w_bar_c:
            # When computing w_bar_c and potentially rho_c, we want to compute w_bar_c over a uniform grid of rho_c values, otherwise just use the given rho_c
            # Forward-simulate the system for n_forward_sim steps
            if compute_rho_c:
                n_rho_c_all = 1000
                rho_c_all = np.linspace(0.01, 100, n_rho_c_all)
                n_forward_sim = n_tmpc - 1
                n_times = 1
            elif compute_w_bar_c:
                n_rho_c_all = 1
                rho_c_all = np.array([rho_c])
                n_forward_sim = 1
                n_times = n_tmpc - n_idx_ignore - n_forward_sim
            x_forward_sim = np.zeros((n_times, 1 + n_forward_sim, nx))
            for t_idx in range(n_times):
                if t_idx < n_times - 1:
                    print(
                        f"Forward simulating time iter {t_idx}/{n_times - 1}", end="\r"
                    )
                else:
                    print(f"Forward simulating time iter {t_idx}/{n_times - 1}")
                if compute_rho_c:
                    x_forward_sim[t_idx, 0] = x_nom_ref[n_idx_ignore + t_idx]
                    for k_idx in range(n_forward_sim):
                        x_forward_sim[t_idx, k_idx + 1] = np.array(
                            helpers.solve_rk4_noise(
                                model.state_update_ct_noise,
                                x_forward_sim[t_idx, k_idx],
                                u_nom_ref[n_idx_ignore + t_idx + k_idx],
                                w_bias,
                                dt_tmpc,
                            )
                        ).flatten()
                elif compute_w_bar_c:
                    x_forward_sim[t_idx, 0] = x_cur_est[n_idx_ignore + t_idx]
                    for k_idx in range(n_forward_sim):
                        x_forward_sim[t_idx, k_idx + 1] = np.array(
                            helpers.solve_rk4_noise(
                                model.state_update_ct_noise,
                                x_forward_sim[t_idx, k_idx],
                                u_pred_traj[n_idx_ignore + t_idx, 0],
                                w_bias,
                                dt_tmpc,
                            )
                        ).flatten()

            # Compute x_err and lyap_err over times and prediction steps
            x_err = np.zeros((n_times, 1 + n_forward_sim, nx))
            lyap_err = np.zeros((n_times, 1 + n_forward_sim))
            for t_idx in range(n_times):
                if t_idx < n_times - 1:
                    print(f"Time iter {t_idx}/{n_times - 1}", end="\r")
                else:
                    print(f"Time iter {t_idx}/{n_times - 1}")
                for k_idx in range(1 + n_forward_sim):
                    # x_err[t_idx, k_idx] = (
                    #     x_cur_est[n_idx_ignore + t_idx + k_idx]
                    #     - x_pred_traj[n_idx_ignore + t_idx, k_idx]
                    # )
                    x_err[t_idx, k_idx] = (
                        x_cur_est[n_idx_ignore + t_idx + k_idx]
                        - x_forward_sim[t_idx, k_idx]
                    )
                    lyap_err[t_idx, k_idx] = np.sqrt(
                        x_err[t_idx, k_idx] @ P_delta @ x_err[t_idx, k_idx]
                    )

            # Compute w_bar_c and corresponding RPI tightening for all rho_c, t, and tau
            w_bar_c_all = np.zeros((n_rho_c_all, n_times, n_forward_sim))
            rpi_tightening_per_rho_c = np.zeros(n_rho_c_all)
            for rho_c_idx, rho_c in enumerate(rho_c_all):
                if rho_c_idx < n_rho_c_all - 1:
                    print(f"rho_c iter {rho_c_idx}/{n_rho_c_all - 1}", end="\r")
                else:
                    print(f"rho_c iter {rho_c_idx}/{n_rho_c_all - 1}")
                for t_idx in range(n_times):
                    for k_idx in range(1, 1 + n_forward_sim):
                        w_bar_c_all[rho_c_idx, t_idx, k_idx - 1] = (
                            lyap_err[t_idx, k_idx]
                            * rho_c
                            / (1 - math.exp(-rho_c * k_idx * dt_tmpc))
                        )
                rpi_tightening_per_rho_c[rho_c_idx] = np.max(
                    w_bar_c_all[rho_c_idx] / rho_c
                )

            # Compute optimal rho_c, if desired, and corresponding w_bar_c
            rho_c_idx = 0
            if compute_rho_c:
                rho_c_idx = np.argmin(rpi_tightening_per_rho_c)
                print(f"Optimal rho_c index: {rho_c_idx}/{n_rho_c_all - 1}")
            rho_c = rho_c_all[rho_c_idx]
            print(f"rho_c: {rho_c}")
            w_bar_c = rpi_tightening_per_rho_c[rho_c_idx] * rho_c
            print(f"w_bar_c: {w_bar_c}")

            # Compute tube size over time
            if compute_rho_c:
                s = helpers.get_tube_size_over_time(
                    1 + n_forward_sim, dt_tmpc, w_bar_c, rho_c
                )

            if compute_w_bar_c:
                files_dict[ros_rec_json_name]["w_bar_c"] = w_bar_c

        # Determine epsilon at all time steps
        if compute_epsilon:
            epsilon_all = np.zeros(n_tmpc)
            for t_idx in range(n_tmpc):
                t_x_cur_idx = np.abs(
                    t_x_cur - t_x_cur_est[n_idx_ignore + t_idx]
                ).argmin()
                x_err = x_cur[t_x_cur_idx] - x_cur_est[n_idx_ignore + t_idx]
                epsilon_all[t_idx] = np.sqrt(x_err.T @ P_delta @ x_err)
            epsilon_all = epsilon_all[n_idx_ignore:]
            epsilon = np.max(epsilon_all)
            files_dict[ros_rec_json_name]["epsilon"] = epsilon
            print(f"epsilon: {epsilon}")

        if compute_x_in_eps_ball:
            if compute_epsilon:
                scaling = np.sqrt(
                    epsilon**2 / (x_in_eps_ball_dir @ P_delta @ x_in_eps_ball_dir)
                )
                x_in_eps_ball = x_in_eps_ball_nom + scaling * x_in_eps_ball_dir
                print(f"x_in_eps_ball: {x_in_eps_ball}")
            else:
                log.warning(
                    "compute_x_in_eps_ball is set to True, but compute_epsilon is False! Cannot compute x_in_eps_ball."
                )
                x_in_eps_ball = None

        # Create rho_c figures per file
        if compute_rho_c:
            # Lyapunov error and corresponding tube over prediction stages
            if do_plot_lyap_err:
                if save_lyap_err:
                    helpers.set_plt_properties()
                    props = helpers.set_fig_properties()
                fig, ax = plt.subplots(
                    figsize=(6.4, 3),
                    num=f"{ros_rec_json_name} - Lyapunov error and tube over prediction stages",
                )
                if not save_lyap_err:
                    fig.suptitle(
                        f"{ros_rec_json_name} - Lyapunov error and tube over prediction stages"
                    )
                ax.plot(
                    np.arange(1 + n_forward_sim),
                    s,
                    linewidth=linewidth,
                    zorder=3,
                    label=r"$s$",
                )
                ax.axhline(
                    y=w_bar_c / rho_c,
                    color="red",
                    linestyle="--",
                    linewidth=linewidth,
                    zorder=2,
                    label=r"$\frac{\bar{w}^\mathrm{c}}{\rho^\mathrm{c}}$",
                )
                ax.plot(
                    np.arange(1 + n_forward_sim),
                    np.max(lyap_err, axis=0),
                    "-.",
                    linewidth=linewidth,
                    zorder=1,
                    label=r"$\sqrt{V^\delta(x_{t+\tau},z_{\tau|t})}$",
                )
                ax.set_xlabel(f"Prediction time $\\tau$")
                ax.set_ylabel(r"$\sqrt{V^\delta(x_{t+\tau},z_{\tau|t})}$")
                if save_lyap_err:
                    ax.xaxis.labelpad = props["xlabelpad"]
                    ax.yaxis.labelpad = props["ylabelpad"]
                    ax.tick_params(pad=props["tickpad"])

                    # Resize figure
                    helpers.resize_fig(fig, scale=1)
                    fig.subplots_adjust(right=0.99, top=0.99, bottom=0.14, left=0.17)

                    # Save figure
                    fig_path = f"{fig_dir}/tube_fit.pdf"
                    helpers.save_fig(fig, fig_path)
                else:
                    ax.legend()

            # RPI tightening over rho_c
            if do_plot_rpi_tightening_over_rho_c:
                fig, ax = plt.subplots()
                fig.suptitle(f"{ros_rec_json_name} - RPI tightening vs. rho_c")
                ax.plot(rho_c_all, rpi_tightening_per_rho_c)
                ax.set_xlabel(r"$\rho^\mathrm{c}$")
                ax.set_ylabel(r"$\frac{\bar{w}^\mathrm{c}}{\rho^\mathrm{c}}$")

            # Create w_bar_c figures per file
            # NOTE: no condition on compute_w_bar_c here, because w_bar_c is always computed with rho_c
            # w_bar_c over time
            if do_plot_w_bar_c_time:
                fig, ax = plt.subplots()
                fig.suptitle(f"{ros_rec_json_name} - computed w_bar_c over time")
                ax.plot(
                    t_x_cur_est[n_idx_ignore + 1 :],
                    w_bar_c_all[rho_c_idx].reshape(
                        -1,
                    ),
                )
                ax.set_xlabel("Time (s)")
                ax.set_ylabel(r"$\bar{w}^\mathrm{c}$")

            # w_bar_c sorted
            if do_plot_w_bar_c_sorted:
                fig, ax = plt.subplots()
                fig.suptitle(f"{ros_rec_json_name} - computed w_bar_c sorted")
                ax.plot(
                    np.arange(n_idx_ignore + 1, n_tmpc),
                    sorted(w_bar_c_all[rho_c_idx].reshape(-1)),
                )
                ax.set_xlabel("Index")
                ax.set_ylabel(r"$\bar{w}^\mathrm{c}$")

        # Create epsilon figures per file
        if compute_epsilon:
            # epsilon over time
            if do_plot_epsilon_time:
                fig, ax = plt.subplots()
                fig.suptitle(f"{ros_rec_json_name} - computed epsilon over time")
                ax.plot(t_x_cur_est[n_idx_ignore:], epsilon_all)
                ax.set_xlabel("Time (s)")
                ax.set_ylabel("$\epsilon$")

            # epsilon sorted
            if do_plot_epsilon_sorted:
                fig, ax = plt.subplots()
                fig.suptitle(f"{ros_rec_json_name} - computed epsilon sorted")
                ax.plot(np.arange(n_idx_ignore, n_tmpc), sorted(epsilon_all))
                ax.set_xlabel("Index")
                ax.set_ylabel("$\epsilon$")

    # Compute maximum w_bar_c and epsilon over all files
    if compute_epsilon:
        epsilon_over_files = np.zeros(n_ros_rec_json_names)
        for file_idx, ros_rec_json_name in enumerate(ros_rec_json_names):
            if ros_rec_json_name in files_dict:
                epsilon_over_files[file_idx] = files_dict[ros_rec_json_name]["epsilon"]
        epsilon = np.max(epsilon_over_files)
        print(f"Maximum epsilon over all files: {epsilon}")

    if compute_w_bar_c:
        w_bar_c_over_files = np.zeros(n_ros_rec_json_names)
        for file_idx, ros_rec_json_name in enumerate(ros_rec_json_names):
            if ros_rec_json_name in files_dict:
                w_bar_c_over_files[file_idx] = files_dict[ros_rec_json_name]["w_bar_c"]
        w_bar_c = np.max(w_bar_c_over_files)
        print(f"Maximum w_bar_c over all files: {w_bar_c}")

    # End timing and print
    end = time.time()
    print(f"Elapsed time: {end - start}")

    plt.show()

    # Save epsilon, rho_c, and w_bar_c to a json file by replacing old data
    with open(output_data_json_path, "r") as file:
        tightening_data = json.load(file)
    if compute_rho_c:
        tightening_data["rho_c"] = rho_c
    if compute_epsilon:
        tightening_data["epsilon"] = epsilon
    if compute_w_bar_c:
        tightening_data["w_bar_c"] = w_bar_c
    with open(output_data_json_path, "w") as file:
        json.dump(tightening_data, file, indent=4)
    print(f"Saved tightening data to {output_data_json_path}")
