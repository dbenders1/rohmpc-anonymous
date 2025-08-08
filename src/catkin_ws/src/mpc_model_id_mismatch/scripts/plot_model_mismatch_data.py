import json
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.ticker import MaxNLocator
from mpc_model_id_mismatch import helpers
from pathlib import Path


def get_eta_est_gt_ratios(eta_min, eta_max, eta_est_all, stage_idx):
    n_iter = eta_est_all.shape[0]
    n_eta = eta_est_all.shape[2]
    eta_est_gt_ratios_min = np.zeros(
        (
            n_iter,
            n_eta,
        )
    )
    eta_est_gt_ratios_max = np.zeros(
        (
            n_iter,
            n_eta,
        )
    )
    for iter_idx in range(n_iter):
        if np.all(eta_min != 0):
            eta_est_gt_ratios_min[iter_idx, :] = (
                np.min(eta_est_all[iter_idx, :, :, stage_idx], axis=0) / eta_min
            )
        else:
            raise ValueError("Cannot compute ratios when eta_min contains zero values.")
        if np.all(eta_max != 0):
            eta_est_gt_ratios_max[iter_idx, :] = (
                np.max(eta_est_all[iter_idx, :, :, stage_idx], axis=0) / eta_max
            )
        else:
            raise ValueError("Cannot compute ratios when eta_max contains zero values.")
    return eta_est_gt_ratios_min, eta_est_gt_ratios_max


def get_likelihood_gt(w, eta, M, time_idx):
    w_sel = w[:, time_idx : time_idx + M]
    eta_sel = eta[:, time_idx : time_idx + M + 1]
    Q_cov = np.cov(w_sel, rowvar=True)
    R_cov = np.cov(eta_sel, rowvar=True)
    Q_mhe = np.linalg.inv(Q_cov)
    R_mhe = np.linalg.inv(R_cov)
    return float(
        helpers.get_cost_mle(
            M,
            w_sel,
            eta_sel,
            Q_mhe,
            R_mhe,
            Q_cov,
            R_cov,
            np.arange(M + 1),
        )
    )


def get_likelihood_over_runs(
    w_est_all, eta_est_all, Q_mhe_all, R_mhe_all, Q_cov_est_all, R_cov_est_all, time_idx
):
    n_iter = w_est_all.shape[0]
    M = w_est_all.shape[3]
    likelihood = np.zeros(n_iter)
    for iter_idx in range(n_iter):
        likelihood[iter_idx] = helpers.get_cost_mle(
            M,
            w_est_all[iter_idx, time_idx, :, :],
            eta_est_all[iter_idx, time_idx, :, :],
            Q_mhe_all[iter_idx, :, :],
            R_mhe_all[iter_idx, :, :],
            Q_cov_est_all[iter_idx, :, :],
            R_cov_est_all[iter_idx, :, :],
            np.arange(M + 1),
        )
    return likelihood


def get_w_est_gt_ratios(w_min, w_max, w_est_all, stage_idx):
    n_iter = w_est_all.shape[0]
    n_w = w_est_all.shape[2]
    w_est_gt_ratios_min = np.zeros(
        (
            n_iter,
            n_w,
        )
    )
    w_est_gt_ratios_max = np.zeros(
        (
            n_iter,
            n_w,
        )
    )
    for iter_idx in range(n_iter):
        if np.all(w_min != 0):
            w_est_gt_ratios_min[iter_idx, :] = (
                np.min(w_est_all[iter_idx, :, :, stage_idx], axis=0) / w_min
            )
        else:
            raise ValueError("Cannot compute ratios when w_min contains zero values.")
        if np.all(w_max != 0):
            w_est_gt_ratios_max[iter_idx, :] = (
                np.max(w_est_all[iter_idx, :, :, stage_idx], axis=0) / w_max
            )
        else:
            raise ValueError("Cannot compute ratios when w_max contains zero values.")
    return w_est_gt_ratios_min, w_est_gt_ratios_max


def get_x_y_fs(model, ts, M, u, x_est, w_est, eta_est, time_idx):
    n_horizon = M + 1
    n_x = x_est.shape[1]
    n_y = n_x
    x_fs = np.zeros((n_horizon, n_x))
    y_fs = np.zeros((n_horizon, n_y))
    x_fs[0, :] = x_est[time_idx, :, 0]
    y_fs[0, :] = model.get_outputs_noise(
        x_fs[0, :], u[:, time_idx], eta_est[time_idx, :, 0]
    )
    for k in range(M):
        x_fs[k + 1, :] = np.array(
            helpers.solve_rk4_noise(
                model.state_update_ct_noise,
                x_est[time_idx, :, k],
                u[:, time_idx + k],
                w_est[time_idx, :, k],
                ts,
            )
        ).reshape((-1,))
        # x_fs[k + 1, :] = np.array(
        #     helpers.solve_rk4(
        #         model.state_update_ct,
        #         x_fs[k, :],
        #         u[:, time_idx + k],
        #         ts,
        #     )
        # ).reshape((-1,))
        # y_fs[k + 1, :] = model.get_outputs(x_fs[k + 1, :], u[:, time_idx + k + 1])
        y_fs[k + 1, :] = model.get_outputs_noise(
            x_fs[k + 1, :], u[:, time_idx + k + 1], eta_est[time_idx, :, k + 1]
        )

    return x_fs, y_fs


def get_thrusts_torques_over_horizon(model, u, x_est, M, time_idx):
    n_horizon = M + 1

    # Compute estimated thrust and torques based on individual rotor thrusts
    thrusts_horizon = np.zeros((n_horizon, 1))
    torques_horizon = np.zeros((n_horizon, 3))
    for k_idx in range(n_horizon):
        thrusts_horizon[k_idx, :] = model.state_update_ct_compute_thrust_t(
            x_est[time_idx, :, k_idx], u[:, time_idx + k_idx]
        )
        torques_horizon[k_idx, :] = np.array(
            model.state_update_ct_compute_torque_t(
                x_est[time_idx, :, k_idx], u[:, time_idx + k_idx]
            )
        ).reshape((-1,))
    return thrusts_horizon, torques_horizon


def plot_y_y_fs_over_horizon(exp_idx, t, y, y_fs, M, time_idx):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Measured outputs vs forward-simulated outputs over horizon",
    )
    fig.suptitle(
        f"Measured outputs vs forward-simulated outputs over horizon at time t={t[time_idx]}s (time_idx={time_idx}/{t.shape[0]})"
    )
    n_horizon = M + 1
    t = (np.arange(0, n_horizon) * ts).reshape((n_horizon,))
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_x_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        if plot_y_idx_at_ax_idx[ax_idx] != None:
            y_idx = plot_y_idx_at_ax_idx[ax_idx]
            axes[row_idx, col_idx].plot(
                t,
                y[y_idx, time_idx : time_idx + n_horizon],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
            axes[row_idx, col_idx].plot(
                t,
                y_fs[:, y_idx],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel(f"{y_labels[y_idx]} ({y_unit_labels[y_idx]})")
    fig.legend(["Measured", "Forward-simulated"])


def plot_y_x_est_u_over_time(exp_idx, t, x_est, y, u, stage_idx):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Measured outputs vs estimated states over time",
    )
    fig.suptitle(
        f"Measured outputs vs estimated states over time at stage k={stage_idx}"
    )
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_x_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        x_idx = plot_x_idx_at_ax_idx[ax_idx]
        if plot_y_idx_at_ax_idx[ax_idx] != None:
            y_idx = plot_y_idx_at_ax_idx[ax_idx]
            axes[row_idx, col_idx].plot(
                t,
                y[y_idx, stage_idx : stage_idx + len(t)],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].plot(
            t,
            x_est[:, x_idx, stage_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        if plot_u_idx_at_ax_idx[ax_idx] != None:
            u_idx = plot_u_idx_at_ax_idx[ax_idx]
            axes[row_idx, col_idx].plot(
                t,
                u[u_idx, stage_idx : stage_idx + len(t)],
                "-+",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel(f"{x_labels[x_idx]} ({x_unit_labels[x_idx]})")
    fig.legend(["Measured outputs", "Estimated states", "Applied inputs"])


def plot_y_x_est_u_over_horizon(exp_idx, t, x_est_all_last_iter, y, u, time_idx):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Measured outputs vs estimated states over horizon",
    )
    fig.suptitle(
        f"Measured outputs vs estimated states over horizon at time t={t[time_idx]}s (time_idx={time_idx}/{t.shape[0]})"
    )
    n_horizon = x_est_all_last_iter.shape[2]
    t = (np.arange(0, n_horizon) * ts).reshape((n_horizon,))
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_x_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        x_idx = plot_x_idx_at_ax_idx[ax_idx]
        if plot_y_idx_at_ax_idx[ax_idx] != None:
            y_idx = plot_y_idx_at_ax_idx[ax_idx]
            axes[row_idx, col_idx].plot(
                t,
                y[y_idx, time_idx : time_idx + n_horizon],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].plot(
            t,
            x_est_all_last_iter[time_idx, x_idx, :],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        if plot_u_idx_at_ax_idx[ax_idx] != None:
            u_idx = plot_u_idx_at_ax_idx[ax_idx]
            axes[row_idx, col_idx].plot(
                t,
                u[u_idx, time_idx : time_idx + n_horizon],
                "-+",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel(f"{x_labels[x_idx]} ({x_unit_labels[x_idx]})")
    fig.legend(["Measured outputs", "Estimated states", "Applied inputs"])


def plot_u_t_over_horizon(exp_idx, t, thrusts_horizon, torques_horizon, M, time_idx):
    n_horizon = M + 1

    # Plot the motor velocity inputs, states, torques and thrust
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Applied inputs over horizon",
    )
    fig.suptitle(
        f"Applied inputs over horizon at time t={t[time_idx]}s (time_idx={time_idx}/{t.shape[0]})"
    )
    t = (np.arange(0, n_horizon) * ts).reshape((n_horizon,))
    for ax_idx in range(n_rows_states * n_cols_states):
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        if ax_idx in [8, 9, 10]:
            axes[row_idx, col_idx].plot(
                t,
                thrusts_horizon,
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        elif ax_idx in [12, 13, 14]:
            axes[row_idx, col_idx].plot(
                t,
                torques_horizon[:, ax_idx - 12],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel("Equiv. inputs")


def plot_u_wm_over_horizon(exp_idx, model, t, x_est, u, M, time_idx):
    n_horizon = M + 1

    # Compute estimated thrust and torques based on motor velocities
    thrust_pred = np.zeros((n_horizon, 1))
    torques_pred = np.zeros((n_horizon, 3))
    for k_idx in range(n_horizon):
        thrust_pred[k_idx, :] = model.state_update_ct_compute_thrust_wm(
            x_est[time_idx, :, k_idx], u[:, time_idx + k_idx]
        )
        torques_pred[k_idx, :] = np.array(
            model.state_update_ct_compute_torque_wm(
                x_est[time_idx, :, k_idx], u[:, time_idx + k_idx]
            )
        ).reshape((-1,))

    # Plot the motor velocity inputs, states, torques and thrust
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Applied inputs over horizon",
    )
    fig.suptitle(
        f"Applied inputs over horizon at time t={t[time_idx]}s (time_idx={time_idx}/{t.shape[0]})"
    )
    t = (np.arange(0, n_horizon) * ts).reshape((n_horizon,))
    for ax_idx in range(n_rows_states * n_cols_states):
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        if ax_idx in [8, 9, 10]:
            axes[row_idx, col_idx].plot(
                t,
                thrust_pred,
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        elif ax_idx in [12, 13, 14]:
            axes[row_idx, col_idx].plot(
                t,
                torques_pred[:, ax_idx - 12],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        elif ax_idx in [16, 17, 18, 19]:
            axes[row_idx, col_idx].plot(
                t,
                u[ax_idx - 16, time_idx : time_idx + n_horizon],
                "-+",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel("Equiv. inputs")


def plot_gazebo_w_over_time(name, w):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=name,
    )
    fig.suptitle(name)
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_w_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        w_idx = plot_w_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].plot(
            w[w_idx, :],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].set_xlabel("Time index")
        axes[row_idx, col_idx].set_ylabel(f"{w_labels[w_idx]} ({w_unit_labels[w_idx]})")


def plot_gazebo_u_w(name, u, w, w_idx):
    n_inputs = u.shape[0]
    fig, axes = plt.subplots(
        n_rows_inputs,
        n_cols_inputs,
        num=f"{name} - Disturbances over inputs",
    )
    fig.suptitle(f"{name} - Disturbances over inputs")
    for ax_idx in range(n_rows_inputs * n_cols_inputs):
        row_idx = ax_idx // n_cols_inputs
        col_idx = ax_idx % n_cols_inputs
        axes[row_idx, col_idx].scatter(
            u[ax_idx, :],
            w[w_idx, :],
            s=sizes,
        )
        axes[row_idx, col_idx].set_xlabel(
            f"{u_labels[ax_idx]} ({u_unit_labels[ax_idx]})"
        )
        axes[row_idx, col_idx].set_ylabel(f"{w_labels[w_idx]} ({w_unit_labels[w_idx]})")
    fig.legend(["Disturbances over inputs"])


def plot_gazebo_y_w(name, y, w, w_idx):
    n_outputs = y.shape[0]
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"{name} - Disturbances over outputs",
    )
    fig.suptitle(f"{name} - Disturbances over outputs")
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_y_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        y_idx = plot_y_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].scatter(
            y[y_idx, :],
            w[w_idx, :],
            s=sizes,
        )
        axes[row_idx, col_idx].set_xlabel(f"{y_labels[y_idx]} ({y_unit_labels[y_idx]})")
        axes[row_idx, col_idx].set_ylabel(f"{w_labels[w_idx]} ({w_unit_labels[w_idx]})")
    fig.legend(["Disturbances over outputs"])


def plot_gazebo_w_sorted(name, w):
    n_samples = w.shape[1]
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"{name} - Disturbances sorted",
    )
    fig.suptitle(f"{name} - Disturbances sorted")
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_w_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        w_idx = plot_w_idx_at_ax_idx[ax_idx]
        w_idc = np.arange(n_samples)
        w_sorted = np.sort(w[w_idx, :])
        axes[row_idx, col_idx].plot(
            w_idc, w_sorted, "-o", linewidth=widths, markersize=sizes
        )
        axes[row_idx, col_idx].set_xlabel("Index")
        axes[row_idx, col_idx].set_ylabel(f"{w_labels[w_idx]} ({w_unit_labels[w_idx]})")
    fig.legend(["Disturbances sorted"])


def plot_w_est_over_time(exp_idx, t, w, w_est, stage_idx):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Estimated disturbances over time",
    )
    fig.suptitle(f"Estimated disturbances over time at stage k={stage_idx}")
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_w_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        w_idx = plot_w_idx_at_ax_idx[ax_idx]
        if w is not None:
            axes[row_idx, col_idx].plot(
                t,
                w[w_idx, stage_idx : stage_idx + len(t)],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].plot(
            t,
            w_est[:, w_idx, stage_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel(f"{w_labels[w_idx]} ({w_unit_labels[w_idx]})")
    if w is not None:
        fig.legend(["Ground truth", "Estimated"])


def plot_w_est_over_horizon(exp_idx, t, ts, w, w_est, time_idx):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Estimated disturbances over horizon",
    )
    fig.suptitle(
        f"Estimated disturbances over horizon at time t={t[time_idx]}s (time_idx={time_idx}/{t.shape[0]})"
    )
    M = w_est.shape[2]
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_w_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        w_idx = plot_w_idx_at_ax_idx[ax_idx]
        if w is not None:
            axes[row_idx, col_idx].plot(
                np.squeeze(np.arange(0, M) * ts),
                w[w_idx, time_idx : time_idx + M],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].plot(
            np.squeeze(np.arange(0, M) * ts),
            w_est[time_idx, w_idx, :],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel(f"{w_labels[w_idx]} ({w_unit_labels[w_idx]})")
    if w is not None:
        fig.legend(["Ground truth", "Estimated"])


def plot_w_est_gt_ratios(exp_idx, w_est_gt_ratios_min, w_est_gt_ratios_max):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Min/max disturbance ratios over iterations",
    )
    fig.suptitle(f"Min/max disturbance ratios over iterations")
    n_iter = w_est_gt_ratios_min.shape[0]
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_w_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        w_idx = plot_w_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].plot(
            np.arange(1, 1 + n_iter),
            w_est_gt_ratios_min[:, w_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].plot(
            np.arange(1, 1 + n_iter),
            w_est_gt_ratios_max[:, w_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].xaxis.set_major_locator(MaxNLocator(integer=True))
        axes[row_idx, col_idx].set_xlabel("Iteration")
        axes[row_idx, col_idx].set_ylabel(f"Disturbance ratio {w_labels[w_idx]}")
    fig.legend(["Min ratios", "Max ratios"])


def plot_w_est_gt_ratios_combined(
    exp_idx,
    w_est_gt_ratios_min,
    w_est_gt_ratios_max,
    do_save_w_est_gt_ratios_combined=False,
):
    if do_save_w_est_gt_ratios_combined:
        helpers.set_plt_properties()
        props = helpers.set_fig_properties()
    fig, ax = plt.subplots(
        figsize=(6.4, 3),
        num=f"Experiment {exp_idx} - Average disturbance ratios over iterations",
    )
    if not do_save_w_est_gt_ratios_combined:
        fig.suptitle(f"Average disturbance ratios over iterations")
    n_iter = w_est_gt_ratios_min.shape[0]
    n_w = w_est_gt_ratios_min.shape[1]
    w_est_gt_ratios_avg = (w_est_gt_ratios_min + w_est_gt_ratios_max) / 2
    gt_ratios_avg = np.mean(w_est_gt_ratios_avg, axis=1)
    gt_ratios_std = np.std(w_est_gt_ratios_avg, axis=1)
    for w_idx in range(n_w):
        ax.plot(
            np.arange(1, 1 + n_iter),
            w_est_gt_ratios_avg[:, w_idx],
            "-.",
            linewidth=widths,
            zorder=0,
            label=f"{w_labels[w_idx]}",
        )
    ax.axhline(
        1.0,
        color="green",
        linestyle="--",
        linewidth=3 * widths,
        zorder=1,
        label="Desired ratio",
    )
    ax.errorbar(
        np.arange(1, 1 + n_iter),
        gt_ratios_avg,
        color="tab:blue",
        yerr=gt_ratios_std,
        elinewidth=3 * widths,
        capsize=5,
        capthick=3 * widths,
        barsabove=True,
        linewidth=3 * widths,
        zorder=2,
        label="Mean and std deviation",
    )
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_yscale("log")
    ax.set_xlabel("Iteration")
    ax.set_ylabel(f"Avg. dist. bounds ratio")
    if do_save_w_est_gt_ratios_combined:
        ax.xaxis.labelpad = props["xlabelpad"]
        ax.yaxis.labelpad = props["ylabelpad"] + 3
        ax.tick_params(pad=props["tickpad"])
        ax.set_axisbelow(True)
        ax.grid(True)

        # Resize figure
        helpers.resize_fig(fig, scale=1)
        fig.subplots_adjust(right=0.99, top=0.99, bottom=0.14, left=0.14)

        # Save figure
        fig_path = f"{fig_dir}/w_est_gt_ratios_combined.pdf"
        helpers.save_fig(fig, fig_path)


def plot_eta_est_over_time(exp_idx, t, eta, eta_est, stage_idx):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Estimated measurement noises over time",
    )
    fig.suptitle(f"Estimated measurement noises over time at stage k={stage_idx}")
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_eta_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        eta_idx = plot_eta_idx_at_ax_idx[ax_idx]
        if eta is not None:
            axes[row_idx, col_idx].plot(
                t,
                eta[eta_idx, stage_idx : stage_idx + len(t)],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].plot(
            t,
            eta_est[:, eta_idx, stage_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel(
            f"{eta_labels[eta_idx]} ({eta_unit_labels[eta_idx]})"
        )
    if eta is not None:
        fig.legend(["Ground truth", "Estimated"])


def plot_eta_est_over_horizon(exp_idx, t, ts, eta, eta_est, time_idx):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Estimated measurement noises over horizon",
    )
    fig.suptitle(
        f"Estimated measurement noises over horizon at time t={t[time_idx]}s (time_idx={time_idx}/{t.shape[0]})"
    )
    n_horizon = eta_est.shape[2]
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_eta_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        eta_idx = plot_eta_idx_at_ax_idx[ax_idx]
        if eta is not None:
            axes[row_idx, col_idx].plot(
                np.squeeze(np.arange(0, n_horizon) * ts),
                eta[eta_idx, time_idx : time_idx + n_horizon],
                "-o",
                linewidth=widths,
                markersize=sizes,
            )
        axes[row_idx, col_idx].plot(
            np.squeeze(np.arange(0, n_horizon) * ts),
            eta_est[time_idx, eta_idx, :],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].set_xlabel("Time (s)")
        axes[row_idx, col_idx].set_ylabel(
            f"{eta_labels[eta_idx]} ({eta_unit_labels[eta_idx]})"
        )
    if eta is not None:
        fig.legend(["Ground truth", "Estimated"])


def plot_eta_est_gt_ratios(exp_idx, eta_est_gt_ratios_min, eta_est_gt_ratios_max):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Min/max measurement noise ratios over iterations",
    )
    fig.suptitle(f"Min/max measurement noise ratios over iterations")
    n_iter = eta_est_gt_ratios_min.shape[0]
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_eta_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        eta_idx = plot_eta_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].plot(
            np.arange(1, 1 + n_iter),
            eta_est_gt_ratios_min[:, eta_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].plot(
            np.arange(1, 1 + n_iter),
            eta_est_gt_ratios_max[:, eta_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].xaxis.set_major_locator(MaxNLocator(integer=True))
        axes[row_idx, col_idx].set_xlabel("Iteration")
        axes[row_idx, col_idx].set_ylabel(
            f"Measurement noise ratio {eta_labels[eta_idx]}"
        )
    fig.legend(["Min ratios", "Max ratios"])


def plot_x_est_w_est_stage(exp_idx, x_est, w_est, stage_idx):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Estimated states vs estimated disturbances over time",
    )
    fig.suptitle(
        f"Estimated states vs estimated disturbances over time at stage k={stage_idx}"
    )
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_w_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        w_idx = plot_w_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].scatter(
            x_est[:, w_idx, stage_idx],
            w_est[:, w_idx, stage_idx],
            s=sizes,
            linewidth=widths,
        )
        axes[row_idx, col_idx].set_xlabel(f"{x_labels[w_idx]} ({x_unit_labels[w_idx]})")
        axes[row_idx, col_idx].set_ylabel(f"{w_labels[w_idx]} ({w_unit_labels[w_idx]})")


def plot_costs(
    exp_idx,
    t,
    costs_w,
    costs_eta,
    costs_term,
    costs_total,
    costs_w_gt,
    costs_eta_gt,
    costs_total_gt,
):
    fig, ax = plt.subplots(1, 1, num=f"Experiment {exp_idx} - Costs over times")
    fig.suptitle(f"Costs over times")
    ax.plot(
        t,
        costs_w,
        "-o",
        linewidth=widths,
        markersize=sizes,
        label="Disturbance contribution to total cost",
    )
    ax.plot(
        t,
        costs_eta,
        "-o",
        linewidth=widths,
        markersize=sizes,
        label="Measurement noise contribution to total cost",
    )
    ax.plot(
        t,
        costs_term,
        "-o",
        linewidth=widths,
        markersize=sizes,
        label="Terminal cost",
    )
    ax.plot(
        t,
        costs_total,
        "-o",
        linewidth=widths,
        markersize=sizes,
        label="Total cost",
    )
    if costs_w_gt is not None:
        ax.plot(
            t,
            costs_w_gt,
            "-o",
            linewidth=widths,
            markersize=sizes,
            label="Disturbance contribution to total ground truth cost",
        )
    if costs_eta_gt is not None:
        ax.plot(
            t,
            costs_eta_gt,
            "-o",
            linewidth=widths,
            markersize=sizes,
            label="Measurement noise contribution to total ground truth cost",
        )
    if costs_total_gt is not None:
        ax.plot(
            t,
            costs_total_gt,
            "-o",
            linewidth=widths,
            markersize=sizes,
            label="Ground truth total cost",
        )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cost")
    fig.legend()


def plot_likelihood(exp_idx, likelihood, likelihood_gt=None, do_save_likelihood=False):
    if do_save_likelihood:
        helpers.set_plt_properties()
        props = helpers.set_fig_properties()
    fig, ax = plt.subplots(
        figsize=(6.4, 3), num=f"Experiment {exp_idx} - Likelihood over iterations"
    )
    if not do_save_likelihood:
        fig.suptitle(f"Likelihood over iterations")
    n_iter = likelihood.shape[0]
    ax.plot(
        np.arange(1, 1 + n_iter),
        -likelihood,
        "-o",
        linewidth=widths,
        markersize=sizes,
        label="MLE",
    )
    if likelihood_gt is not None:
        ax.axhline(
            y=-likelihood_gt,
            color="red",
            linestyle="--",
            linewidth=widths,
            label="Ground truth likelihood",
        )
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xlabel("Iteration")
    ax.set_ylabel("log likelihood")
    if do_save_likelihood:
        ax.xaxis.labelpad = props["xlabelpad"]
        ax.yaxis.labelpad = props["ylabelpad"]
        ax.tick_params(pad=props["tickpad"])
        ax.set_axisbelow(True)
        ax.grid(True)

        # Resize figure
        helpers.resize_fig(fig, scale=1)
        fig.subplots_adjust(right=0.99, top=0.99, bottom=0.14, left=0.14)

        # Save figure
        fig_path = f"{fig_dir}/likelihood.pdf"
        helpers.save_fig(fig, fig_path)


def plot_Q_R_trace(exp_idx, Q_cov_est_all, R_cov_est_all):
    fig, axes = plt.subplots(
        1,
        2,
        num=f"Experiment {exp_idx} - Trace of Q and R matrices over iterations",
    )
    fig.suptitle(f"Trace of Q and R matrices over iterations")
    fig.subplots_adjust(wspace=0.4)
    n_iter = Q_cov_est_all.shape[0]
    axes[0].plot(
        np.arange(1, 1 + n_iter),
        np.trace(Q_cov_est_all, axis1=1, axis2=2),
        "-o",
        linewidth=widths,
        markersize=sizes,
    )
    axes[0].set_title("trace(Q)")
    axes[0].xaxis.set_major_locator(MaxNLocator(integer=True))
    axes[0].set_xlabel("Iteration")
    axes[0].set_ylabel("Value")
    axes[1].plot(
        np.arange(1, 1 + n_iter),
        np.trace(R_cov_est_all, axis1=1, axis2=2),
        "-o",
        linewidth=widths,
        markersize=sizes,
    )
    axes[1].set_title("trace(R)")
    axes[1].xaxis.set_major_locator(MaxNLocator(integer=True))
    axes[1].set_xlabel("Iteration")
    axes[1].set_ylabel("Value")


def plot_Q_diag(exp_idx, Q_cov_est_all):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Diagonal of Q matrix over iterations",
    )
    fig.suptitle(f"Diagonal of Q matrix over iterations")
    n_iter = Q_cov_est_all.shape[0]
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_w_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        w_idx = plot_w_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].plot(
            np.arange(1, 1 + n_iter),
            Q_cov_est_all[:, w_idx, w_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].xaxis.set_major_locator(MaxNLocator(integer=True))
        axes[row_idx, col_idx].set_xlabel("Iteration")
        axes[row_idx, col_idx].set_ylabel(f"Q[{w_idx}, {w_idx}]")


def plot_R_diag(exp_idx, R_cov_est_all):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Diagonal of R matrix over iterations",
    )
    fig.suptitle(f"Diagonal of R matrix over iterations")
    n_iter = R_cov_est_all.shape[0]
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_eta_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        eta_idx = plot_eta_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].plot(
            np.arange(1, 1 + n_iter),
            R_cov_est_all[:, eta_idx, eta_idx],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].xaxis.set_major_locator(MaxNLocator(integer=True))
        axes[row_idx, col_idx].set_xlabel("Iteration")
        axes[row_idx, col_idx].set_ylabel(f"R[{eta_idx}, {eta_idx}]")


def plot_Q_eig_vals(exp_idx, Q_cov_est_all):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Eigenvalues of Q matrix over iterations",
    )
    fig.suptitle(f"Eigenvalues of Q matrix over iterations")
    n_iter = Q_cov_est_all.shape[0]
    eig_vals = np.zeros((Q_cov_est_all.shape[1], Q_cov_est_all.shape[0]))
    for i in range(Q_cov_est_all.shape[0]):
        eig_vals[:, i] = np.linalg.eigvals(Q_cov_est_all[i, :, :])
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_w_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        w_idx = plot_w_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].plot(
            np.arange(1, 1 + n_iter),
            eig_vals[w_idx, :],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].xaxis.set_major_locator(MaxNLocator(integer=True))
        axes[row_idx, col_idx].set_xlabel("Iteration")
        axes[row_idx, col_idx].set_ylabel(f"Eigenvalue {w_idx}")


def plot_R_eig_vals(exp_idx, R_cov_est_all):
    fig, axes = plt.subplots(
        n_rows_states,
        n_cols_states,
        num=f"Experiment {exp_idx} - Eigenvalues of R matrix over iterations",
    )
    fig.suptitle(f"Eigenvalues of R matrix over iterations")
    n_iter = R_cov_est_all.shape[0]
    eig_vals = np.zeros((R_cov_est_all.shape[1], R_cov_est_all.shape[0]))
    for i in range(R_cov_est_all.shape[0]):
        eig_vals[:, i] = np.linalg.eigvals(R_cov_est_all[i, :, :])
    for ax_idx in range(n_rows_states * n_cols_states):
        if plot_eta_idx_at_ax_idx[ax_idx] == None:
            axes.flat[ax_idx].axis("off")
            continue
        row_idx = ax_idx // n_cols_states
        col_idx = ax_idx % n_cols_states
        eta_idx = plot_eta_idx_at_ax_idx[ax_idx]
        axes[row_idx, col_idx].plot(
            np.arange(1, 1 + n_iter),
            eig_vals[eta_idx, :],
            "-o",
            linewidth=widths,
            markersize=sizes,
        )
        axes[row_idx, col_idx].xaxis.set_major_locator(MaxNLocator(integer=True))
        axes[row_idx, col_idx].set_xlabel("Iteration")
        axes[row_idx, col_idx].set_ylabel(f"Eigenvalue {eta_idx}")


if __name__ == "__main__":
    # Ensure that matrices are fully printed
    np.set_printoptions(threshold=np.inf)

    # User settings
    package_dir = Path(__file__).parents[1]
    config_dir = f"{package_dir}/config"
    config_path = f"{config_dir}/scripts/plot_model_mismatch_data.yaml"
    data_dir = f"{package_dir}/data"
    model_mismatch_results_dir = f"{data_dir}/model_mismatch_results"
    fig_dir = f"{data_dir}/figures/model_mismatch"

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    # Get common parameters
    mhe_json_name = config["mhe_json_name"]
    time_idx = config["time_idx"]

    gazebo_w_json_names = config["gazebo_w_json_names"]
    gazebo_w_idx = config["gazebo_w_idx"]

    do_plot_y_y_fs_over_horizon = config["do_plot"]["y_y_fs_over_horizon"]
    do_plot_y_x_est_u_over_time = config["do_plot"]["y_x_est_u_over_time"]
    do_plot_y_x_est_u_over_horizon = config["do_plot"]["y_x_est_u_over_horizon"]
    do_plot_u_t_over_horizon = config["do_plot"]["u_t_over_horizon"]
    do_plot_u_wm_over_horizon = config["do_plot"]["u_wm_over_horizon"]
    do_plot_w_est_over_time = config["do_plot"]["w_est_over_time"]
    do_plot_w_est_over_horizon = config["do_plot"]["w_est_over_horizon"]
    do_plot_eta_est_over_time = config["do_plot"]["eta_est_over_time"]
    do_plot_eta_est_over_horizon = config["do_plot"]["eta_est_over_horizon"]
    do_plot_x_est_w_est_stage = config["do_plot"]["x_est_w_est_stage"]
    do_plot_w_est_gt_ratios = config["do_plot"]["w_est_gt_ratios"]
    do_plot_w_est_gt_ratios_combined = config["do_plot"]["w_est_gt_ratios_combined"]
    do_plot_eta_est_gt_ratios = config["do_plot"]["eta_est_gt_ratios"]
    do_plot_costs = config["do_plot"]["costs"]
    do_plot_likelihood = config["do_plot"]["likelihood"]
    do_plot_Q_R_trace = config["do_plot"]["Q_R_trace"]
    do_plot_Q_diag = config["do_plot"]["Q_diag"]
    do_plot_R_diag = config["do_plot"]["R_diag"]
    do_plot_Q_eig_vals = config["do_plot"]["Q_eig_vals"]
    do_plot_R_eig_vals = config["do_plot"]["R_eig_vals"]

    sizes = config["plot_settings"]["sizes"]
    widths = config["plot_settings"]["widths"]
    n_rows_inputs = config["plot_settings"]["n_rows_inputs"]
    n_cols_inputs = config["plot_settings"]["n_cols_inputs"]
    n_rows_states = config["plot_settings"]["n_rows_states"]
    n_cols_states = config["plot_settings"]["n_cols_states"]
    plot_x_idx_at_ax_idx = config["plot_settings"]["plot_x_idx_at_ax_idx"]
    plot_y_idx_at_ax_idx = config["plot_settings"]["plot_y_idx_at_ax_idx"]
    plot_u_idx_at_ax_idx = config["plot_settings"]["plot_u_idx_at_ax_idx"]
    plot_w_idx_at_ax_idx = config["plot_settings"]["plot_w_idx_at_ax_idx"]
    plot_eta_idx_at_ax_idx = config["plot_settings"]["plot_eta_idx_at_ax_idx"]
    u_labels = config["plot_settings"]["u_labels"]
    u_unit_labels = config["plot_settings"]["u_unit_labels"]
    x_labels = config["plot_settings"]["x_labels"]
    x_unit_labels = config["plot_settings"]["x_unit_labels"]
    y_labels = config["plot_settings"]["y_labels"]
    y_unit_labels = config["plot_settings"]["y_unit_labels"]
    w_labels = config["plot_settings"]["w_labels"]
    w_unit_labels = config["plot_settings"]["w_unit_labels"]
    eta_labels = config["plot_settings"]["eta_labels"]
    eta_unit_labels = config["plot_settings"]["eta_unit_labels"]

    do_save_w_est_gt_ratios_combined = config["save_settings"][
        "w_est_gt_ratios_combined"
    ]
    if do_save_w_est_gt_ratios_combined:
        do_plot_w_est_gt_ratios_combined = True
    do_save_likelihood = config["save_settings"]["likelihood"]
    if do_save_likelihood:
        do_plot_likelihood = True

    # Read data
    with open(f"{model_mismatch_results_dir}/{mhe_json_name}.json", "r") as f:
        data = json.load(f)

    # Get common data
    data_common = data["common"]
    g = data_common["g"]
    ts = data_common["ts"]
    params_file = data_common["params_file"]
    model_name = data_common["model_name"]
    M = data_common["M"]
    stage_est = data_common["stage_est"]
    nx = data_common["nx"]
    nw = data_common["nw"]
    neta = data_common["neta"]

    # Select experiments
    exp_idc = [0]
    exp_names = []
    for key in data.keys():
        if key not in ["__header__", "__version__", "__globals__", "common"]:
            exp_names.append(key)
    if len(exp_idc) > len(exp_names):
        raise ValueError(
            f"Number of selected experiments ({len(exp_idc)}) exceeds number of available experiments ({len(exp_names)})"
        )
    for exp_idx in exp_idc:
        exp_name = exp_names[exp_idx]
        print(f"Experiment {exp_idx}: {exp_name}")

        # Get experiment data
        data_exp = data[exp_name]
        t = np.array(data_exp["t"]).reshape((-1,))
        y = np.array(data_exp["y"])
        u = np.array(data_exp["u"])
        costs_total = np.array(data_exp["costs_total"])
        costs_term = np.array(data_exp["costs_term"])
        costs_w = np.array(data_exp["costs_w"])
        costs_eta = np.array(data_exp["costs_eta"])
        w = None
        if "w" in data_exp:
            w = np.array(data_exp["w"])
            w_min = np.array(data_exp["w_min"])
            w_max = np.array(data_exp["w_max"])
            costs_w_gt = np.array(data_exp["costs_w_gt"])
        eta = None
        if "eta" in data_exp:
            eta = np.array(data_exp["eta"])
            eta_min = np.array(data_exp["eta_min"])
            eta_max = np.array(data_exp["eta_max"])
            costs_eta_gt = np.array(data_exp["costs_eta_gt"])
        if "w" in data_exp and "eta" in data_exp:
            costs_total_gt = np.array(data_exp["costs_total_gt"])
        x_est_all = np.array(data_exp["x_est_all"])
        w_est_all = np.array(data_exp["w_est_all"])
        eta_est_all = np.array(data_exp["eta_est_all"])
        Q_mhe_all = np.array(data_exp["Q_mhe_all"])
        R_mhe_all = np.array(data_exp["R_mhe_all"])
        Q_cov_est_all = np.array(data_exp["Q_cov_est_all"])
        R_cov_est_all = np.array(data_exp["R_cov_est_all"])
        n_iter = x_est_all.shape[0]
        n_t = x_est_all.shape[1]
        t = t[stage_est : stage_est + n_t]

        # Create model class
        model = helpers.DroneAgiModel(model_name, g, params_file)

        # Set data indices to use
        iter_idx = n_iter - 1
        stage_idx = stage_est

        # Select data for the selected iteration
        x_est_all_last_iter = x_est_all[iter_idx, :, :, :]
        w_est_all_last_iter = w_est_all[iter_idx, :, :, :]
        eta_est_all_last_iter = eta_est_all[iter_idx, :, :, :]
        costs_total = costs_total[iter_idx, :].reshape((-1,))
        costs_term = costs_term[iter_idx, :].reshape((-1,))
        costs_w = costs_w[iter_idx, :].reshape((-1,))
        costs_eta = costs_eta[iter_idx, :].reshape((-1,))
        if w is not None:
            costs_w_gt = costs_w_gt[iter_idx, :].reshape((-1,))
        if eta is not None:
            costs_eta_gt = costs_eta_gt[iter_idx, :].reshape((-1,))
        if w is not None and eta is not None:
            costs_total_gt = costs_total_gt[iter_idx, :].reshape((-1,))

        # Create plots
        if do_plot_y_y_fs_over_horizon:
            x_fs, y_fs = get_x_y_fs(
                model,
                ts,
                M,
                u,
                x_est_all_last_iter,
                w_est_all_last_iter,
                eta_est_all_last_iter,
                time_idx,
            )
            plot_y_y_fs_over_horizon(exp_idx, t, y, y_fs, M, time_idx)
        if do_plot_y_x_est_u_over_time:
            plot_y_x_est_u_over_time(exp_idx, t, x_est_all_last_iter, y, u, stage_idx)
        if do_plot_y_x_est_u_over_horizon:
            plot_y_x_est_u_over_horizon(exp_idx, t, x_est_all_last_iter, y, u, time_idx)
        if do_plot_u_t_over_horizon:
            thrusts_horizon, torques_horizon = get_thrusts_torques_over_horizon(
                model, u, x_est_all_last_iter, M, time_idx
            )
            plot_u_t_over_horizon(
                exp_idx, t, thrusts_horizon, torques_horizon, M, time_idx
            )
        if do_plot_u_wm_over_horizon:
            plot_u_wm_over_horizon(
                exp_idx, model, t, x_est_all_last_iter, u, M, time_idx
            )
        if do_plot_w_est_over_time:
            plot_w_est_over_time(exp_idx, t, w, w_est_all_last_iter, stage_idx)
        if do_plot_w_est_over_horizon:
            plot_w_est_over_horizon(exp_idx, t, ts, w, w_est_all_last_iter, time_idx)
        if do_plot_w_est_gt_ratios:
            if w is not None:
                w_est_gt_ratios_min, w_est_gt_ratios_max = get_w_est_gt_ratios(
                    w_min, w_max, w_est_all, stage_idx
                )
                plot_w_est_gt_ratios(exp_idx, w_est_gt_ratios_min, w_est_gt_ratios_max)
        if do_plot_w_est_gt_ratios_combined:
            if w is not None:
                w_est_gt_ratios_min, w_est_gt_ratios_max = get_w_est_gt_ratios(
                    w_min, w_max, w_est_all, stage_idx
                )
                plot_w_est_gt_ratios_combined(
                    exp_idx,
                    w_est_gt_ratios_min,
                    w_est_gt_ratios_max,
                    do_save_w_est_gt_ratios_combined,
                )
        if do_plot_eta_est_over_time:
            plot_eta_est_over_time(exp_idx, t, eta, eta_est_all_last_iter, stage_idx)
        if do_plot_eta_est_over_horizon:
            plot_eta_est_over_horizon(
                exp_idx, t, ts, eta, eta_est_all_last_iter, time_idx
            )
        if do_plot_eta_est_gt_ratios:
            if eta is not None:
                eta_est_gt_ratios_min, eta_est_gt_ratios_max = get_eta_est_gt_ratios(
                    eta_min, eta_max, eta_est_all, stage_idx
                )
                plot_eta_est_gt_ratios(
                    exp_idx, eta_est_gt_ratios_min, eta_est_gt_ratios_max
                )
        if do_plot_x_est_w_est_stage:
            plot_x_est_w_est_stage(
                exp_idx, x_est_all_last_iter, w_est_all_last_iter, stage_idx
            )
        if do_plot_costs:
            plot_costs(
                exp_idx,
                t,
                costs_w,
                costs_eta,
                costs_term,
                costs_total,
                costs_w_gt,
                costs_eta_gt,
                costs_total_gt,
            )
        if do_plot_likelihood:
            likelihood = get_likelihood_over_runs(
                w_est_all,
                eta_est_all,
                Q_mhe_all,
                R_mhe_all,
                Q_cov_est_all,
                R_cov_est_all,
                time_idx,
            )
            if w is not None and eta is not None:
                likelihood_gt = get_likelihood_gt(w, eta, M, time_idx)
            else:
                likelihood_gt = None
            plot_likelihood(exp_idx, likelihood, None, do_save_likelihood)
        if do_plot_Q_R_trace:
            plot_Q_R_trace(exp_idx, Q_cov_est_all, R_cov_est_all)
        if do_plot_Q_diag:
            plot_Q_diag(exp_idx, Q_cov_est_all)
        if do_plot_R_diag:
            plot_R_diag(exp_idx, R_cov_est_all)
        if do_plot_Q_eig_vals:
            plot_Q_eig_vals(exp_idx, Q_cov_est_all)
        if do_plot_R_eig_vals:
            plot_R_eig_vals(exp_idx, R_cov_est_all)

    if gazebo_w_json_names:
        for gazebo_w_json_name in gazebo_w_json_names:
            with open(
                f"{model_mismatch_results_dir}/{gazebo_w_json_name}.json", "r"
            ) as f:
                gazebo_w_data = json.load(f)
            u = np.array(gazebo_w_data["u"])
            y = np.array(gazebo_w_data["y"])
            w = np.array(gazebo_w_data["w"])
            plot_gazebo_w_over_time(gazebo_w_json_name, w)
            plot_gazebo_u_w(gazebo_w_json_name, u, w, gazebo_w_idx)
            plot_gazebo_y_w(gazebo_w_json_name, y, w, gazebo_w_idx)
            plot_gazebo_w_sorted(gazebo_w_json_name, w)

    plt.show()
