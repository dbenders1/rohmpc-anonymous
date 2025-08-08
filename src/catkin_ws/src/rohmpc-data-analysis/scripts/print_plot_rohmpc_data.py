import argparse
import json
import logging
import yaml

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np

from matplotlib.patches import Circle, Polygon
from os import path
from pathlib import Path
from rohmpc_data_analysis import helpers


np.set_printoptions(threshold=np.inf)


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
    config_path = f"{config_dir}/scripts/print_plot_rohmpc_data.yaml"
    fig_dir = f"{package_dir}/data/figures"

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
    runtime_json_name = config["data"]["runtime_json_name"]
    ros_rec_json_name = config["data"]["ros_rec_json_name"]

    print_settings = config["print_settings"]
    t_print = print_settings["t"]
    check_gt_est_states = print_settings["check_gt_est_states"]
    print_pobj_comp = print_settings["print_pobj_comp"]
    print_track_err = print_settings["print_track_err"]

    do_plot_settings = config["do_plot"]
    do_plot_obs = do_plot_settings["obs"]
    do_plot_obs_inflation = do_plot_settings["obs_inflation"]
    do_plot_pmpc_tube = do_plot_settings["pmpc_tube"]
    do_plot_pmpc_pred = do_plot_settings["pmpc_pred"]
    do_plot_tmpc_ref = do_plot_settings["tmpc_ref"]
    do_plot_tmpc_tube = do_plot_settings["tmpc_tube"]
    do_plot_tmpc_pred = do_plot_settings["tmpc_pred"]
    do_plot_x = do_plot_settings["x"]

    plot_settings = config["plot_settings"]
    t_plot = plot_settings["t"]
    k_pmpc_pred = plot_settings["k_pmpc_pred"]
    k_ref = plot_settings["k_ref"]
    k_pred = plot_settings["k_pred"]
    t_div_tmpc_tube = plot_settings["t_div_tmpc_tube"]
    linewidth_pmpc_tube = plot_settings["linewidth_pmpc_tube"]
    linewidth_tmpc_tube = plot_settings["linewidth_tmpc_tube"]
    grid_resolution = plot_settings["grid_resolution"]
    half_grid_resolution = grid_resolution / 2
    r_pmpc = plot_settings["r_pmpc"]
    r_tmpc_ref = plot_settings["r_tmpc_ref"]
    r_tmpc = plot_settings["r_tmpc"]
    r_x = plot_settings["r_x"]
    alpha_tmpc_tube_controller = plot_settings["alpha_tmpc_tube_controller"]
    alpha_tmpc_tube_total = plot_settings["alpha_tmpc_tube_total"]

    save_settings = config["save_settings"]
    show_fig = save_settings["show_fig"]
    save_fig = save_settings["save_fig"]
    fig_name = save_settings["fig_name"]

    c_obs_inflated = helpers.Colors.GREY.value
    c_pmpc = mcolors.CSS4_COLORS["blue"]
    c_tmpc_ref = mcolors.CSS4_COLORS["red"]
    c_tmpc = helpers.Colors.ORANGE.value
    c_x = mcolors.CSS4_COLORS["green"]
    c_xf = mcolors.CSS4_COLORS["black"]
    zorder_obs = 0
    zorder_pmpc_tube = 1
    zorder_pmpc_pred = 2
    zorder_tmpc_ref = 3
    zorder_tmpc_tube_controller = 4
    zorder_tmpc_tube_total = 5
    zorder_tmpc_pred = 6
    zorder_x = 7
    alpha_max = 1
    alpha_min = 0.5

    # Read ROS runtime json data
    runtime_json_path = f"{runtime_json_dir}/{runtime_json_name}.json"
    log.warning(f"Selected runtime json file: {runtime_json_path}")
    if not path.exists(runtime_json_path):
        raise FileNotFoundError(f"Runtime json path {runtime_json_path} not found")
    with open(runtime_json_path, "r") as file:
        runtime_data = json.load(file)

    runtime_static_data = runtime_data["static_data"]
    Q = np.array(runtime_static_data["Q"])
    R = np.array(runtime_static_data["R"])
    P = np.array(runtime_static_data["P"])
    alpha = runtime_static_data["alpha"]
    c_o = runtime_static_data["c_o"]
    epsilon = runtime_static_data["epsilon"]
    robot_radius = runtime_static_data["robot_radius"]
    s_pred = np.array(runtime_static_data["s_pred"])
    stepsize_tmpc = runtime_static_data["stepsize"]
    steps_tmpc = runtime_static_data["steps"]

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

    obs_len = 0.6
    obs_dim = np.array([2, 1]) * obs_len
    data_obs_0 = ros_rec_data["/grid/obs/rec/rectangle2x1_0"]
    obs_0_p = np.array(data_obs_0["p"])
    obs_0_eul = np.array(data_obs_0["eul"])
    obs_0 = {"p": obs_0_p, "eul": obs_0_eul, "dim": obs_dim}

    data_obs_1 = ros_rec_data["/grid/obs/rec/rectangle2x1_1"]
    obs_1_p = np.array(data_obs_1["p"])
    obs_1_eul = np.array(data_obs_1["eul"])
    obs_1 = {"p": obs_1_p, "eul": obs_1_eul, "dim": obs_dim}
    obs_list = [obs_0, obs_1]

    data_x_cur = ros_rec_data["/falcon/ground_truth/odometry"]
    t_x_cur = np.array(data_x_cur["t"])
    x_cur = np.array(data_x_cur["x"])

    data_x_cur_est = ros_rec_data["/mpc/rec/current_state"]
    t_x_cur_est = np.array(data_x_cur_est["t"])
    x_cur_est = np.array(data_x_cur_est["current_state"])

    data_ref_traj = ros_rec_data["/mpc/rec/reference_trajectory/0"]
    t_ref_traj = np.array(data_ref_traj["t"])
    u_ref_traj = np.array(data_ref_traj["u_ref"])
    x_ref_traj = np.array(data_ref_traj["x_ref"])

    data_pred_traj = ros_rec_data["/mpc/rec/predicted_trajectory/0"]
    t_pred_traj = np.array(data_pred_traj["t"])
    u_pred_traj = np.array(data_pred_traj["u_pred"])
    x_pred_traj = np.array(data_pred_traj["x_pred"])

    data_pmpc_pred_traj = ros_rec_data["/mpc/rec/predicted_trajectory/1"]
    t_pmpc_pred_traj = np.array(data_pmpc_pred_traj["t"])
    u_pmpc_pred_traj = np.array(data_pmpc_pred_traj["u_pred"])
    x_pmpc_pred_traj = np.array(data_pmpc_pred_traj["x_pred"])

    # Computed vertices of obstacle and vertices required to create inflated rectangular obstacles
    obs_verts = helpers.compute_inflated_obstacle_vertices(
        obs_list, robot_radius - half_grid_resolution
    )

    # Set times to a specific precision
    t_x_cur = np.round(t_x_cur, time_precision)
    t_x_cur_est = np.round(t_x_cur_est, time_precision)
    t_ref_traj = np.round(t_ref_traj, time_precision)
    t_pred_traj = np.round(t_pred_traj, time_precision)
    t_pmpc_pred_traj = np.round(t_pmpc_pred_traj, time_precision)

    # Determine various parameters of runtime data
    n_tmpc = min(len(t_x_cur_est), len(t_ref_traj), len(t_pred_traj))
    N_pmpc = x_pmpc_pred_traj.shape[1] - 1
    N_tmpc = x_ref_traj.shape[1] - 1
    dt_pmpc = np.round(t_pmpc_pred_traj[1] - t_pmpc_pred_traj[0], time_precision)
    dt_tmpc = steps_tmpc * stepsize_tmpc
    ts_pmpc = dt_pmpc
    ts_tmpc = np.round(t_ref_traj[1] - t_ref_traj[0], time_precision)
    ts_sim = np.round(t_x_cur[1] - t_x_cur[0], time_precision)
    Q_dt = Q * dt_tmpc
    R_dt = R * dt_tmpc

    # Shift pmpc predicted trajectory times ts_pmpc forward (since the state is valid from one step into the future)
    t_pmpc_pred_traj += ts_pmpc

    # Ensure that all times are aligned
    if t_x_cur_est[-1] > t_x_cur[-1]:
        log.warning(
            "The estimated state has a recording after the ground truth state. Shrinking (t_x_cur_est, x_cur_est), (t_ref_traj, u_ref_traj, x_ref_traj), and (t_pred_traj, u_pred_traj, x_pred_traj) to the last time of t_x_cur"
        )
        t_x_cur_est = t_x_cur_est[t_x_cur_est <= t_x_cur[-1]]
        x_cur_est = x_cur_est[: len(t_x_cur_est)]
        t_ref_traj = t_ref_traj[t_ref_traj <= t_x_cur[-1]]
        u_ref_traj = u_ref_traj[: len(t_ref_traj)]
        x_ref_traj = x_ref_traj[: len(t_ref_traj)]
        t_pred_traj = t_pred_traj[t_pred_traj <= t_x_cur[-1]]
        u_pred_traj = u_pred_traj[: len(t_pred_traj)]
        x_pred_traj = x_pred_traj[: len(t_pred_traj)]
        t_pmpc_pred_traj = t_pmpc_pred_traj[t_pmpc_pred_traj <= t_x_cur[-1]]
        u_pmpc_pred_traj = u_pmpc_pred_traj[: len(t_pmpc_pred_traj)]
        x_pmpc_pred_traj = x_pmpc_pred_traj[: len(t_pmpc_pred_traj)]

    # Check if the estimated state overlaps with the ground truth state
    if check_gt_est_states:
        x_cur_mpc_start = x_cur[np.where(np.isin(t_x_cur, t_x_cur_est))[0], :]
        states_equal = np.all(
            np.abs(x_cur_mpc_start - x_cur_est) < helpers.FLOAT_TOL, axis=1
        )
        if np.all(states_equal):
            log.warning(
                "The estimated state matches the ground truth state at the MPC start times"
            )
        else:
            log.warning(
                "The estimated state does not match the ground truth state at the MPC start times"
            )
            first_diff_idx = np.where(~states_equal)[0][0]
            if first_diff_idx < x_cur_mpc_start.shape[0]:
                log.warning(
                    f"First difference at index {first_diff_idx}: "
                    f"First difference at time {t_x_cur_est[first_diff_idx]} s: "
                    f"x_cur_mpc_start: {x_cur_mpc_start[first_diff_idx]}, "
                    f"x_cur_est: {x_cur_est[first_diff_idx]}"
                )

    # Determine min and max times for printing and plotting
    t_min = min(np.min(t_x_cur_est), np.min(t_ref_traj), np.min(t_pred_traj))
    t_max = max(np.max(t_x_cur_est), np.max(t_ref_traj), np.max(t_pred_traj))

    # Select data to print
    if t_print != -1 and (t_print < t_min or t_print > t_max):
        raise ValueError(
            f"t_print ({t_print}) should be either -1 or be contained within bounds [{t_min}, {t_max}]"
        )

    if t_print < 0:
        t_x_cur_est_idx = -1
        t_pred_traj_idx = -1
        t_ref_traj_idx = -1
    else:
        t_x_cur_est_idx = np.abs(t_x_cur_est - t_print).argmin()
        t_mpc_start = t_x_cur_est[t_x_cur_est_idx]
        t_x_cur_start_idx = np.abs(t_x_cur - t_mpc_start).argmin()
        t_x_cur_end_idx = np.abs(
            t_x_cur - np.round(t_mpc_start + N_tmpc * ts_tmpc, time_precision)
        ).argmin()
        t_pred_traj_idx = np.abs(t_pred_traj - t_print).argmin()
        t_ref_traj_idx = np.abs(t_ref_traj - t_print).argmin()

    # Compute objective values
    if print_pobj_comp:
        if t_x_cur_est_idx < 0:
            pobj_comp = np.zeros(n_tmpc)
            for t_idx in range(n_tmpc):
                pobj_comp[t_idx] = helpers.compute_pobj(
                    u_pred_traj[t_idx, :, :],
                    x_pred_traj[t_idx, :, :],
                    u_ref_traj[t_idx, :, :],
                    x_ref_traj[t_idx, :, :],
                    N_tmpc,
                    Q_dt,
                    R_dt,
                    P,
                )
        else:
            pobj_comp = helpers.compute_pobj(
                u_pred_traj[t_pred_traj_idx, :, :],
                x_pred_traj[t_pred_traj_idx, :, :],
                u_ref_traj[t_ref_traj_idx, :, :],
                x_ref_traj[t_ref_traj_idx, :, :],
                N_tmpc,
                Q_dt,
                R_dt,
                P,
            )
        print(f"pobj_comp: {pobj_comp}")

    # Compute position tracking error
    if print_track_err:
        for key in print_track_err:
            if key == "total":
                u_idc = np.arange(helpers.NU)
                x_idc = np.arange(helpers.NX)
            elif key == "u":
                u_idc = np.arange(helpers.NU)
                x_idc = []
            elif key == "x":
                u_idc = []
                x_idc = np.arange(helpers.NX)
            elif key == "p":
                u_idc = []
                x_idc = np.arange(3)
            elif key == "2d_p":
                u_idc = []
                x_idc = np.arange(2)
            else:
                raise ValueError(f"Unknown track error type '{key}'")
            if t_x_cur_est_idx < 0:
                track_err = np.zeros(n_tmpc)
                for t_idx in range(n_tmpc):
                    track_err[t_idx] = helpers.compute_track_err(
                        u_pred_traj[t_idx, :, :],
                        x_pred_traj[t_idx, :, :],
                        u_ref_traj[t_idx, :, :],
                        x_ref_traj[t_idx, :, :],
                        N_tmpc,
                        u_idc,
                        x_idc,
                    )
            else:
                track_err = helpers.compute_track_err(
                    u_pred_traj[t_pred_traj_idx, :, :],
                    x_pred_traj[t_pred_traj_idx, :, :],
                    u_ref_traj[t_ref_traj_idx, :, :],
                    x_ref_traj[t_ref_traj_idx, :, :],
                    N_tmpc,
                    u_idc,
                    x_idc,
                )
            print(f"{key}_track_err: {track_err}")

    # Select data to plot
    if len(t_plot) != 2:
        raise ValueError("t_plot must contain exactly two elements")
    elif (
        t_plot[0] < t_min or t_plot[0] > t_max or t_plot[1] < t_min or t_plot[1] > t_max
    ):
        raise ValueError(
            f"The interval specified by t_plot ({t_plot}) must be contained within bounds [{t_min}, {t_max}]"
        )
    t_total_plot = t_plot[1] - t_plot[0]
    mpc_idc_plot = np.where((t_x_cur_est >= t_plot[0]) & (t_x_cur_est <= t_plot[1]))[0]
    t_x_cur_est_plot = t_x_cur_est[mpc_idc_plot]
    x_cur_est_plot = x_cur_est[mpc_idc_plot]
    t_ref_traj_plot = t_ref_traj[mpc_idc_plot]
    u_ref_traj_plot = u_ref_traj[mpc_idc_plot]
    x_ref_traj_plot = x_ref_traj[mpc_idc_plot]
    t_pred_traj_plot = t_pred_traj[mpc_idc_plot]
    u_pred_traj_plot = u_pred_traj[mpc_idc_plot]
    x_pred_traj_plot = x_pred_traj[mpc_idc_plot]
    x_cur_idc_plot = np.where(
        (t_x_cur >= t_x_cur_est_plot[0]) & (t_x_cur <= t_x_cur_est_plot[-1])
    )[0]
    t_x_cur_plot = t_x_cur[x_cur_idc_plot]
    x_cur_plot = x_cur[x_cur_idc_plot]
    pmpc_to_plot = np.where(
        (t_pmpc_pred_traj >= t_plot[0]) & (t_pmpc_pred_traj <= t_plot[1])
    )[0]
    t_pmpc_pred_traj_plot = t_pmpc_pred_traj[pmpc_to_plot]
    u_pmpc_pred_traj_plot = u_pmpc_pred_traj[pmpc_to_plot]
    x_pmpc_pred_traj_plot = x_pmpc_pred_traj[pmpc_to_plot]

    # Create figure
    helpers.set_plt_properties()
    props = helpers.set_fig_properties()
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    handles = []

    # Add obstacles to plot
    if do_plot_obs:
        handles_obs = []
        for obs_idx, verts in enumerate(obs_verts):
            if obs_idx == 0:
                obs_polygon = Polygon(
                    verts["orig"],
                    closed=True,
                    edgecolor="black",
                    facecolor="black",
                    label="Obstacle",
                )
            else:
                obs_polygon = Polygon(
                    verts["orig"], closed=True, edgecolor="black", facecolor="black"
                )
            handles_obs.append(ax.add_patch(obs_polygon))
        handles.append(handles_obs[0])
    if do_plot_obs_inflation:
        handles_obs_inflation = []
        for obs_idx, verts in enumerate(obs_verts):
            if obs_idx == 0:
                obs_polygon_long = Polygon(
                    verts["long"],
                    closed=True,
                    color=c_obs_inflated,
                    zorder=zorder_obs,
                    label="Obstacle inflation",
                )
            else:
                obs_polygon_long = Polygon(
                    verts["long"], closed=True, color=c_obs_inflated, zorder=zorder_obs
                )
            handles_obs_inflation.append(ax.add_patch(obs_polygon_long))
            obs_polygon_wide = Polygon(
                verts["wide"], closed=True, color=c_obs_inflated, zorder=zorder_obs
            )
            handles_obs_inflation.append(ax.add_patch(obs_polygon_wide))
            for i in range(4):
                obs_polygon_circle = Circle(
                    verts["orig"][i, :],
                    robot_radius - half_grid_resolution,
                    color=c_obs_inflated,
                    fill=True,
                    zorder=zorder_obs,
                )
                handles_obs_inflation.append(ax.add_patch(obs_polygon_circle))
        handles.append(handles_obs_inflation[0])

    # Compute PMPC tube around reference trajectory and add to plot
    if do_plot_pmpc_tube:
        ref_traj_pos = x_ref_traj_plot[:, 0, :2]
        pmpc_tightening = c_o * alpha
        pmpc_tube = helpers.compute_tube(ref_traj_pos, pmpc_tightening)
        handles_pmpc_tube = []
        for tube_idx in range(pmpc_tube.shape[0]):
            tube_handle = ax.plot(
                pmpc_tube[tube_idx, :, 0],
                pmpc_tube[tube_idx, :, 1],
                color=c_pmpc,
                linewidth=linewidth_pmpc_tube,
                zorder=zorder_pmpc_tube,
                label="PMPC $\\alpha$-tube pos",
            )
            handles_pmpc_tube.append(tube_handle)
        handles.append(handles_pmpc_tube[0][0])

    # Add PMPC prediction to plot
    if do_plot_pmpc_pred:
        handles_pmpc_pred = []
        for t_idx in pmpc_to_plot:
            for k in k_pmpc_pred:
                pmpc_pred = Circle(
                    (
                        x_pmpc_pred_traj[t_idx, k, 0],
                        x_pmpc_pred_traj[t_idx, k, 1],
                    ),
                    r_pmpc,
                    facecolor=c_pmpc,
                    alpha=helpers.compute_alpha(
                        alpha_min, alpha_max, k * ts_pmpc, t_total_plot
                    ),
                    zorder=zorder_pmpc_pred,
                    label="PMPC predicted pos",
                )
                handles_pmpc_pred.append(ax.add_patch(pmpc_pred))
        handles.append(handles_pmpc_pred[0])

    # Add TMPC reference trajectory to plot
    if do_plot_tmpc_ref:
        handles_tmpc_ref = []
        for t_idx in mpc_idc_plot:
            for k in k_ref:
                tmpc_ref = Circle(
                    (
                        x_ref_traj[t_idx, k, 0],
                        x_ref_traj[t_idx, k, 1],
                    ),
                    r_tmpc_ref,
                    facecolor=c_tmpc_ref,
                    alpha=helpers.compute_alpha(
                        alpha_min, alpha_max, k * ts_tmpc, t_total_plot
                    ),
                    zorder=zorder_tmpc_ref,
                    label="TMPC ref pos",
                )
                handles_tmpc_ref.append(ax.add_patch(tmpc_ref))
        handles.append(handles_tmpc_ref[0])

    # Compute growing tubes around specific TMPC predictions and add to plot
    if do_plot_tmpc_tube:
        handles_tmpc_tube = []
        pred_traj_pos_tube = x_pred_traj_plot[::t_div_tmpc_tube, :, :2]
        for t_idx in range(pred_traj_pos_tube.shape[0]):
            tube_controller = helpers.compute_tube(
                pred_traj_pos_tube[t_idx, :, :],
                c_o * s_pred,
            )
            tube_total = helpers.compute_tube(
                pred_traj_pos_tube[t_idx, :, :],
                c_o * (s_pred + epsilon),
            )
            # Plot the shaded area of the controller tube
            handles_tmpc_tube_controller = []
            for i in range(tube_controller.shape[0] - 1):
                for j in range(tube_controller.shape[1] - 1):
                    area = [
                        tube_controller[i, j],
                        tube_controller[i, j + 1],
                        tube_controller[i + 1, j + 1],
                        tube_controller[i + 1, j],
                    ]
                    polygon = Polygon(
                        area,
                        facecolor=c_tmpc,
                        edgecolor="none",
                        alpha=alpha_tmpc_tube_controller,
                        zorder=zorder_tmpc_tube_controller,
                        closed=True,
                        label="TMPC $s$-tube pos",
                    )
                    handles_tmpc_tube_controller.append(ax.add_patch(polygon))
            # Plot the shaded area between the controller and total tube
            handles_tmpc_tube_total = []
            for i in range(tube_controller.shape[0]):
                for j in range(tube_controller.shape[1] - 1):
                    area = [
                        tube_controller[i, j],
                        tube_controller[i, j + 1],
                        tube_total[i, j + 1],
                        tube_total[i, j],
                    ]
                    polygon = Polygon(
                        area,
                        facecolor=c_tmpc,
                        edgecolor="none",
                        alpha=alpha_tmpc_tube_total,
                        zorder=zorder_tmpc_tube_total,
                        closed=True,
                        label="TMPC $(s+\\epsilon)$-tube pos",
                    )
                    handles_tmpc_tube_total.append(ax.add_patch(polygon))
        handles.append(handles_tmpc_tube_controller[0])
        handles.append(handles_tmpc_tube_total[0])

    # Add TMPC prediction to plot
    if do_plot_tmpc_pred:
        handles_tmpc_pred = []
        for t_idx in mpc_idc_plot:
            for k in k_pred:
                tmpc_pred = Circle(
                    (
                        x_pred_traj[t_idx, k, 0],
                        x_pred_traj[t_idx, k, 1],
                    ),
                    r_tmpc,
                    facecolor=c_tmpc,
                    alpha=helpers.compute_alpha(
                        alpha_min, alpha_max, k * ts_tmpc, t_total_plot
                    ),
                    zorder=zorder_tmpc_pred,
                    label="TMPC predicted pos",
                )
                handles_tmpc_pred.append(ax.add_patch(tmpc_pred))
        handles.append(handles_tmpc_pred[0])

    # Add closed-loop state to plot
    if do_plot_x:
        handles_x = []
        for t_idx in x_cur_idc_plot:
            x = Circle(
                (x_cur[t_idx, 0], x_cur[t_idx, 1]),
                r_x,
                facecolor=c_x,
                alpha=helpers.compute_alpha(
                    alpha_min,
                    alpha_max,
                    (t_idx - x_cur_idc_plot[0]) * ts_sim,
                    t_total_plot,
                ),
                zorder=zorder_x,
                label="Closed-loop pos",
            )
            handles_x.append(ax.add_patch(x))
        handles.append(handles_x[0])

    # Add title, etc. to plot
    ax.set_xlim(-1.29, -1.08)
    ax.set_ylim(-0.74, -0.63)
    ax.set_xlabel("$p^x$ (m)")
    ax.set_ylabel("$p^y$ (m)")
    ax.xaxis.labelpad = props["xlabelpad"]
    ax.yaxis.labelpad = props["ylabelpad"]
    ax.tick_params(pad=props["tickpad"])
    ax.set_axisbelow(True)
    ax.legend(
        handles=handles,
        handler_map={Circle: helpers.HandlerCircle()},
        loc="upper right",
        bbox_to_anchor=(0.98, 1),
    )
    ax.grid(True)
    ax.set_aspect("equal", adjustable="box")

    # Resize figure
    helpers.resize_fig(fig, scale=1)
    fig.subplots_adjust(right=0.99, top=0.99, left=0.14, bottom=0.14)

    # Save figures
    if save_fig:
        fig_path = f"{fig_dir}/{fig_name}.pdf"
        helpers.save_fig(fig, fig_path)

    # Show figures
    if show_fig:
        plt.show()
