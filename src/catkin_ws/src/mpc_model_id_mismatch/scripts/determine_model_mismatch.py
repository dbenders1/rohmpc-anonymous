import argparse
import json
import logging
import time

import matplotlib.pyplot as plt
import numpy as np
import scipy.io
import yaml

from mpc_model_id_mismatch import helpers
from pathlib import Path
from os import path
from scipy import interpolate, stats
from scipy.linalg import block_diag
from sklearn.metrics import mean_squared_error

# Logging functionality
log = logging.getLogger(__name__)


class ComputeModelMismatch:
    def __init__(
        self,
        config,
        json_dir,
        json_name,
        data_sel_dir,
        data_sel_file_name,
        output_data_dir,
        exp_type,
        model,
        determine_w,
        use_predetermined_w,
        solver,
        sim_w_max,
        sim_eta_max,
    ) -> None:
        # Process config
        self.overwrite_data_sel = config["recorded_data"]["data_sel"]["overwrite"]
        self.automatic_data_sel = config["recorded_data"]["data_sel"]["automatic"]
        self.automatic_data_sel_first_offset_dist = config["recorded_data"]["data_sel"][
            "automatic_first_offset_dist"
        ]
        self.ts = config["recorded_data"]["processing"]["ts"]

        self.model_name = config["model"]["name"]
        self.quad_number = config["model"]["quad_number"]

        self.mhe_n_iter = config["mhe"]["n_iter"]
        self.mhe_n_times = config["mhe"]["n_times"]
        self.M = config["mhe"]["M"]
        self.stage_est = config["mhe"]["stage_est"]
        self.eps = float(config["mhe"]["eps"])
        self.cost_scaling = float(config["mhe"]["cost_scaling"])
        self.update_Q = config["mhe"]["update_Q"]
        self.update_R = config["mhe"]["update_R"]

        self.do_print_disturbances_min = config["printing"]["disturbances"]["min"]
        self.do_print_disturbances_max = config["printing"]["disturbances"]["max"]
        self.do_print_disturbances_bias = config["printing"]["disturbances"]["bias"]
        self.do_print_disturbances_min_rmse = config["printing"]["disturbances"][
            "min_rmse"
        ]
        self.do_print_disturbances_max_rmse = config["printing"]["disturbances"][
            "max_rmse"
        ]
        self.do_print_disturbances_total_rmse = config["printing"]["disturbances"][
            "total_rmse"
        ]
        self.do_print_meas_noises_min = config["printing"]["meas_noises"]["min"]
        self.do_print_meas_noises_max = config["printing"]["meas_noises"]["max"]

        self.do_plot_raw_interp_inputs = config["plotting"]["raw_interp_inputs"]
        self.do_plot_raw_interp_outputs = config["plotting"]["raw_interp_outputs"]
        self.do_plot_raw_interp_disturbances = config["plotting"][
            "raw_interp_disturbances"
        ]
        self.do_plot_raw_interp_meas_noises = config["plotting"][
            "raw_interp_meas_noises"
        ]

        # Process other arguments
        self.json_dir = json_dir
        self.json_name = json_name
        self.data_sel_dir = data_sel_dir
        self.data_sel_file_name = data_sel_file_name
        self.output_data_dir = output_data_dir
        self.exp_type = exp_type
        self.sim_w_max = sim_w_max
        self.sim_eta_max = sim_eta_max

        # Variables to indicate whether ground truth data is available
        self.disturbances_gt_known = False
        self.measurement_noises_gt_known = False

        # Set model
        self.model = model
        self.n_inputs = self.model.get_n_inputs()
        self.n_states = self.model.get_n_states()
        self.n_outputs = self.model.get_n_outputs()
        self.n_hidden_states = self.model.get_n_hidden_states()
        self.output_idc = self.model.get_output_idc()
        self.hidden_state_idc = self.model.get_hidden_state_idc()
        self.n_disturbances = self.model.get_n_disturbances()
        self.disturbance_idc = self.model.get_disturbance_idc()
        self.n_measurement_noises = self.model.get_n_measurement_noises()
        self.F = self.model.get_measurement_noise_prop_matrix()
        self.F_transpose = self.model.get_measurement_noise_sel_matrix()

        # Set solver
        self.solver = solver

        # Handle MHE settings
        if self.mhe_n_iter < 1:
            log.warning(
                f"Number of MHE iterations ({self.mhe_n_iter}) is less than 1. Setting it to 1"
            )
            self.mhe_n_iter = 1

        # Handle settings related to determining w
        self.determine_w = determine_w
        self.use_predetermined_w = use_predetermined_w
        if self.determine_w:
            if self.exp_type == "gaz":
                if self.mhe_n_iter > 1:
                    log.warning(
                        f"Only one MHE iteration is required to determine w. Setting 'mhe_n_iter' to 1"
                    )
                self.mhe_n_iter = 1
        if self.determine_w or self.use_predetermined_w:
            self.w_json_path = f"{self.output_data_dir}/{self.json_name}_w.json"

    def process_recorded_data(self):
        # READ RECORDED DATA
        # -------------------------------------------------------------------------------
        with open(f"{self.json_dir}/{self.json_name}.json", "r") as openfile:
            json_data = json.load(openfile)
        self.time_precision = json_data["time_precision"]

        self.inputs_times = np.array(json_data["/step_control"]["t"])
        wmc = np.array(json_data["/step_control"]["u"]).T
        n_inputs_times = len(self.inputs_times)
        self.inputs = np.zeros((4, n_inputs_times))
        for i in range(n_inputs_times):
            self.inputs[:, i] = self.model.motor_speeds_to_thrusts(wmc[:, i])

        self.outputs_times = np.array(json_data["/falcon/odometry"]["t"])
        self.outputs = np.array(json_data["/falcon/odometry"]["y"]).T

        self.disturbances_times = np.array(json_data["/w"]["t"])
        self.disturbances = np.array(json_data["/w"]["w"]).T
        if self.disturbances_times.size > 0:
            self.disturbances_gt_known = True

        self.measurement_noises_times = np.array(json_data["/eta"]["t"])
        self.measurement_noises = np.array(json_data["/eta"]["eta"]).T
        if self.measurement_noises_times.size > 0:
            self.measurement_noises_gt_known = True
        # -------------------------------------------------------------------------------

        # REMOVE ALL NON-IDEALITIES FROM TIME VECTORS
        # NOTE: this is necessary to avoid floating point precision errors to have an effect on the zero model mismatch results
        # -------------------------------------------------------------------------------
        # Round time arrays
        self.inputs_times = np.round(self.inputs_times, self.time_precision)
        self.outputs_times = np.round(self.outputs_times, self.time_precision)
        if self.disturbances_gt_known:
            self.disturbances_times = np.round(
                self.disturbances_times, self.time_precision
            )
        if self.measurement_noises_gt_known:
            self.measurement_noises_times = np.round(
                self.measurement_noises_times, self.time_precision
            )
        # -------------------------------------------------------------------------------

        # SELECT DATA
        # -------------------------------------------------------------------------------
        # Check for existing data selection entry corresponding to this json data file in the json file
        data_sel_path = f"{self.data_sel_dir}/{self.data_sel_file_name}"
        with open(data_sel_path, "r") as openfile:
            select_data_dict = json.load(openfile)

        if (self.json_name in select_data_dict) and not self.overwrite_data_sel:
            sel = select_data_dict[self.json_name]
        else:
            if self.automatic_data_sel:
                # Select data between the indices of the crossings at (0,0) indicated in the yaml file
                # Apply moving average to smooth the output data and remove head and tail
                window_size = 10
                x_ma = np.convolve(
                    self.outputs[0, :], np.ones(window_size) / window_size, mode="same"
                )
                x_ma = x_ma[window_size - 1 : -window_size + 1]
                y_ma = np.convolve(
                    self.outputs[1, :], np.ones(window_size) / window_size, mode="same"
                )
                y_ma = y_ma[window_size - 1 : -window_size + 1]

                # Determine distances of all positions to (0,0)
                distances = np.sqrt(x_ma**2 + y_ma**2)

                # Select the distances from the first moment that we are far enough away from (0,0)
                idx_offset = np.where(
                    distances > self.automatic_data_sel_first_offset_dist
                )[0][0]
                distances = distances[idx_offset:]

                # Determine the crossings: the points where the distance gradient changes sign from negative to positive
                distances_gradient = np.gradient(distances, self.ts)

                # In case of noisy measurements: select the first high positive gradient of the gradient since some time (above 4 standard deviations of distribution based on double gradient)
                distances_gradient_gradient = np.gradient(distances_gradient, self.ts)

                # Compute indices of positive outliers in distances_gradient_gradient
                z_scores = np.abs(stats.zscore(distances_gradient_gradient))
                outlier_indices = np.where(z_scores > 4)[0]

                # Remove subsequent outlier indices
                outlier_indices_diff = np.diff(outlier_indices)
                crossing_idc = (
                    idx_offset
                    + outlier_indices[
                        np.append(
                            [0], np.where(outlier_indices_diff > 1 / self.ts)[0] + 1
                        )
                    ]
                )

                # Determine automatic_data_sel_crossing_idc_to_sel based on the type of trajectory
                if "circle" in self.json_name:
                    automatic_data_sel_crossing_idc_to_sel = [
                        0,
                        1,
                    ]  # use for circle
                elif "lemniscate" in self.json_name:
                    automatic_data_sel_crossing_idc_to_sel = [
                        0,
                        2,
                    ]  # use for lemniscate
                else:
                    raise ValueError(
                        f"Unknown trajectory type in json_name: {self.json_name} for automatically selecting data"
                    )

                # Determine the selected points
                min_n_crossings = automatic_data_sel_crossing_idc_to_sel[1] + 1
                if np.isscalar(crossing_idc):
                    crossing_idc = np.array([crossing_idc])
                n_crossings = len(crossing_idc)
                if n_crossings < min_n_crossings:
                    print(
                        f"WARNING: Automatic data selection: expected at least {min_n_crossings} detected crossings, but only found {n_crossings}. Exiting"
                    )
                    exit(1)
                else:
                    if n_crossings > min_n_crossings:
                        print(
                            f"INFO: Automatic data selection: expected at least {min_n_crossings} detected crossings, found {n_crossings}"
                        )
                    crossing_idx_0 = crossing_idc[
                        automatic_data_sel_crossing_idc_to_sel[0]
                    ]
                    crossing_idx_1 = crossing_idc[
                        automatic_data_sel_crossing_idc_to_sel[1]
                    ]
                    sel = [
                        (
                            self.outputs_times[crossing_idx_0],
                            self.outputs[0, crossing_idx_0],
                        ),
                        (
                            self.outputs_times[crossing_idx_1],
                            self.outputs[0, crossing_idx_1],
                        ),
                    ]
                    print(f"Automatically selected points: {sel}")
            else:
                fig, ax = plt.subplots(2, 1)
                fig.suptitle(f"Select part of the interpolated data")
                ax[0].plot(self.outputs_times, self.outputs[0, :], label="x")
                ax[0].plot(self.outputs_times, self.outputs[1, :], label="y")
                ax[0].plot(self.outputs_times, self.outputs[2, :], label="z")
                ax[0].legend()
                ax[0].set_ylabel("Amplitude (m)")
                ax[1].plot(self.inputs_times, self.inputs[0, :], label="t0c")
                ax[1].plot(self.inputs_times, self.inputs[1, :], label="t1c")
                ax[1].plot(self.inputs_times, self.inputs[2, :], label="t2c")
                ax[1].plot(self.inputs_times, self.inputs[3, :], label="t3c")
                ax[1].set_ylabel("Amplitude (N)")
                ax[1].legend()
                ax[1].set_xlabel("Time (s)")
                sel = plt.ginput(2, show_clicks=True)
                print(f"Manually selected points: {sel}")
                print("You can close the data selection figure now")

            plt.figure()
            outputs_idc = np.where(
                np.logical_and(
                    self.outputs_times >= sel[0][0], self.outputs_times <= sel[1][0]
                )
            )[0]
            plt.plot(
                self.outputs_times[outputs_idc], self.outputs[0, outputs_idc], label="x"
            )
            plt.plot(
                self.outputs_times[outputs_idc], self.outputs[1, outputs_idc], label="y"
            )
            plt.title("Selected data")
            plt.xlabel("Time (s)")
            plt.ylabel("Amplitude (m)")
            plt.legend()
            plt.show()

            select_data_dict[self.json_name] = sel
            with open(data_sel_path, "w") as outfile:
                json.dump(select_data_dict, outfile)
            print(f"Added sel for {self.json_name} to {self.data_sel_file_name}")

        # Select the data in the corresponding arrays
        inputs_idc = np.where(
            np.logical_and(
                self.inputs_times >= sel[0][0], self.inputs_times <= sel[1][0]
            )
        )[0]
        self.inputs_times = self.inputs_times[inputs_idc]
        self.inputs = self.inputs[:, inputs_idc]

        outputs_idc = np.where(
            np.logical_and(
                self.outputs_times >= sel[0][0], self.outputs_times <= sel[1][0]
            )
        )[0]
        self.outputs_times = self.outputs_times[outputs_idc]
        self.outputs = self.outputs[:, outputs_idc]

        if self.disturbances_gt_known:
            disturbances_idc = np.where(
                np.logical_and(
                    self.disturbances_times >= sel[0][0],
                    self.disturbances_times <= sel[1][0],
                )
            )[0]
            self.disturbances_times = self.disturbances_times[disturbances_idc]
            self.disturbances = self.disturbances[:, disturbances_idc]

        if self.measurement_noises_gt_known:
            measurement_noises_idc = np.where(
                np.logical_and(
                    self.measurement_noises_times >= sel[0][0],
                    self.measurement_noises_times <= sel[1][0],
                )
            )[0]
            self.measurement_noises_times = self.measurement_noises_times[
                measurement_noises_idc
            ]
            self.measurement_noises = self.measurement_noises[:, measurement_noises_idc]
        # -------------------------------------------------------------------------------

        # INTERPOLATE DATA
        # -------------------------------------------------------------------------------
        self.times_max_begin = max(self.outputs_times[0], self.inputs_times[0])
        self.times_min_end = min(self.outputs_times[-1], self.inputs_times[-1])
        self.times_int = np.arange(self.times_max_begin, self.times_min_end, self.ts)
        self.times_int = np.round(
            self.times_int, self.time_precision
        )  # NOTE: same argument as before with all time vectors

        f = interpolate.interp1d(self.inputs_times, self.inputs, kind="previous")
        self.inputs_int = f(self.times_int)
        f = interpolate.interp1d(self.outputs_times, self.outputs)
        self.outputs_int = f(self.times_int)

        if self.disturbances_gt_known:
            f = interpolate.interp1d(self.disturbances_times, self.disturbances)
            self.disturbances_int = f(self.times_int)

        if self.measurement_noises_gt_known:
            f = interpolate.interp1d(
                self.measurement_noises_times, self.measurement_noises
            )
            self.measurement_noises_int = f(self.times_int)

        self.times_int = self.times_int - self.times_int[0]
        self.n_times = len(self.times_int)
        # -------------------------------------------------------------------------------

        # IF DETERMINING W: REMOVE MEASUREMENT NOISE FROM OUTPUTS
        # -------------------------------------------------------------------------------
        if self.determine_w:
            if self.measurement_noises_gt_known:
                self.outputs_int = (
                    self.outputs_int - self.F @ self.measurement_noises_int
                )
                self.measurement_noises_int = np.zeros(self.measurement_noises.shape)
            else:
                log.warning(
                    "Measurement noises are not known. Cannot remove them from outputs. Therefore, the disturbance vector w cannot be determined"
                )
                exit(1)
        # -------------------------------------------------------------------------------

        # IF USING PREDETERMINED W: LOAD FROM FILE AND SET CORRESPONDING VARIABLES
        # -------------------------------------------------------------------------------
        if self.use_predetermined_w:
            if path.exists(self.w_json_path):
                self.disturbances_gt_known = True
                with open(self.w_json_path, "r") as w_file:
                    w_dict = json.load(w_file)
                    self.disturbances_int = np.array(w_dict["w"])
                    print(
                        f"Loaded interpolated disturbance data from {self.w_json_path}"
                    )
            else:
                raise FileNotFoundError(
                    f"File {self.w_json_path} does not exist. Please run the script with 'determine_w: true' to create it"
                )
        # -------------------------------------------------------------------------------

    def compute_model_mismatch_mhe(self):
        # Based on code here: https://gitlab.ethz.ch/ics/parametric-mhe/-/blob/main/parametric-mhe.ipynb

        # Determine number of MHE time steps
        if self.mhe_n_times < 1:
            self.mhe_n_times = self.n_times
        elif self.mhe_n_times <= self.n_times - self.M:
            self.mhe_n_times = self.M + self.mhe_n_times
        else:
            raise ValueError(
                f"Number of MHE time steps ({self.mhe_n_times}) is larger than the maximum allowed number of time steps ({self.n_times - self.M})"
            )

        # Cut disturbances and measurement noises to the number of MHE time steps
        if self.disturbances_gt_known:
            if self.disturbances_int.shape[1] > self.mhe_n_times - 1:
                self.disturbances_int = self.disturbances_int[:, : self.mhe_n_times - 1]
            elif self.disturbances_int.shape[1] < self.mhe_n_times - 1:
                raise ValueError(
                    f"Number of MHE time steps - 1 ({self.mhe_n_times - 1}) is larger than the number of disturbance time steps ({self.disturbances_int.shape[1]})"
                )
        if self.measurement_noises_gt_known:
            if self.measurement_noises_int.shape[1] > self.mhe_n_times:
                self.measurement_noises_int = self.measurement_noises_int[
                    :, : self.mhe_n_times
                ]
            elif self.measurement_noises_int.shape[1] < self.mhe_n_times:
                raise ValueError(
                    f"Number of MHE time steps ({self.mhe_n_times}) is larger than the number of measurement noise time steps ({self.measurement_noises_int.shape[1]})"
                )

        # Covariance matrices of disturbances (Q_cov) and measurement noises (R_cov)
        # Need to empirically determine if these matrices have converged by plotting their values using plot_model_mismatch_data.py
        self.Q_cov_est_all = np.zeros(
            (self.mhe_n_iter + 1, self.n_disturbances, self.n_disturbances)
        )
        self.R_cov_est_all = np.zeros(
            (self.mhe_n_iter + 1, self.n_measurement_noises, self.n_measurement_noises)
        )

        # Initialize Q_cov and R_cov
        self.Q_cov_est_all[0, :, :] = np.eye(self.n_disturbances)
        self.R_cov_est_all[0, :, :] = np.eye(self.n_measurement_noises)

        # Initialize parameters, cost and initial guess
        p = np.zeros((self.n_inputs + self.n_outputs,))
        if self.determine_w:
            yref = np.zeros((self.n_states,))
        else:
            yref = np.zeros((self.n_disturbances + self.n_measurement_noises,))
        x_warmstart = np.zeros(
            (self.mhe_n_iter, self.mhe_n_times, self.n_states, self.M + 1)
        )
        # Set initial guess for the first time step in the first iteration
        # Initial states equal the measured outputs => start from non-zero disturbance values
        for k in range(self.M + 1):
            x_warmstart[0, 0, self.output_idc, k] = self.outputs_int[:, k]
            x_warmstart[0, 0, self.hidden_state_idc, k] = np.zeros(
                (self.n_hidden_states,)
            )
        if self.determine_w:
            u_warmstart = np.zeros(
                (self.mhe_n_iter, self.mhe_n_times, self.n_states, self.M)
            )
        else:
            u_warmstart = np.zeros(
                (self.mhe_n_iter, self.mhe_n_times, self.n_disturbances, self.M)
            )

        # Create storage for saving optimization results
        self.Q_mhe_all = np.zeros(
            (self.mhe_n_iter, self.n_disturbances, self.n_disturbances)
        )
        self.R_mhe_all = np.zeros(
            (self.mhe_n_iter, self.n_measurement_noises, self.n_measurement_noises)
        )
        self.x_mhe_all = np.zeros(
            (self.mhe_n_iter, self.mhe_n_times - self.M, self.n_states, self.M + 1)
        )
        if self.determine_w:
            self.w_mhe_all = np.zeros(
                (self.mhe_n_iter, self.mhe_n_times - self.M, self.n_states, self.M)
            )
        else:
            self.w_mhe_all = np.zeros(
                (
                    self.mhe_n_iter,
                    self.mhe_n_times - self.M,
                    self.n_disturbances,
                    self.M,
                )
            )
        self.eta_mhe_all = np.zeros(
            (
                self.mhe_n_iter,
                self.mhe_n_times - self.M,
                self.n_measurement_noises,
                self.M + 1,
            )
        )
        self.costs_total = np.zeros((self.mhe_n_iter, self.mhe_n_times - self.M))
        self.costs_term = np.zeros((self.mhe_n_iter, self.mhe_n_times - self.M))
        self.costs_w = np.zeros((self.mhe_n_iter, self.mhe_n_times - self.M))
        self.costs_eta = np.zeros((self.mhe_n_iter, self.mhe_n_times - self.M))
        if self.disturbances_gt_known:
            self.costs_w_gt = np.zeros((self.mhe_n_iter, self.mhe_n_times - self.M))
        if self.measurement_noises_gt_known:
            self.costs_eta_gt = np.zeros((self.mhe_n_iter, self.mhe_n_times - self.M))
        if self.disturbances_gt_known and self.measurement_noises_gt_known:
            self.costs_total_gt = np.zeros((self.mhe_n_iter, self.mhe_n_times - self.M))

        # Iteratively find w_est, eta_est, Q and R
        for i in range(self.mhe_n_iter):
            print(f"\nIteration {i + 1}/{self.mhe_n_iter}")

            # Print eigenvalues of Q_cov and R_cov used for this iteration
            print(
                f"Eigenvalues Q_cov_est_all[{i}, :, :]: {np.linalg.eigvals(self.Q_cov_est_all[i, :, :])}"
            )
            print(
                f"Eigenvalues R_cov_est_all[{i}, :, :]: {np.linalg.eigvals(self.R_cov_est_all[i, :, :])}"
            )

            # Compute Q and R weighting matrices
            self.Q_mhe_all[i, :, :], self.R_mhe_all[i, :, :] = (
                helpers.get_mhe_weighting_matrices(
                    self.Q_cov_est_all[i, :, :], self.R_cov_est_all[i, :, :], self.eps
                )
            )

            # Perform MHE from index self.M onwards
            for t in range(self.M, self.mhe_n_times):
                if t < self.mhe_n_times - 1:
                    print(
                        f"Time step {t - self.M + 1}/{self.mhe_n_times - self.M}",
                        end="\r",
                    )
                else:
                    print(f"Time step {t - self.M + 1}/{self.mhe_n_times - self.M}")

                # if self.determine_w:
                #     # Set initial state constraint
                #     self.solver.set(0, "lbx", self.outputs_int[:, t - self.M])
                #     self.solver.set(0, "ubx", self.outputs_int[:, t - self.M])

                # Set up problem stages 0 - self.M-1
                for k in range(self.M):
                    # Update cost terms
                    if self.determine_w:
                        W = block_diag(
                            10 * np.eye(self.n_states - self.n_disturbances),
                            self.Q_mhe_all[i, :, :],
                        )
                    else:
                        W = block_diag(self.Q_mhe_all[i, :, :], self.R_mhe_all[i, :, :])
                    self.solver.cost_set(k, "W", W, api="new")
                    self.solver.cost_set(k, "yref", yref)
                    self.solver.cost_set(k, "scaling", self.cost_scaling)

                    # Update parameters
                    p = np.concatenate(
                        (
                            self.inputs_int[:, t - self.M + k],
                            self.outputs_int[:, t - self.M + k],
                        )
                    )
                    self.solver.set(k, "p", p)

                    # Update initial guess
                    self.solver.set(k, "x", x_warmstart[i, t - self.M, :, k])
                    self.solver.set(k, "u", u_warmstart[i, t - self.M, :, k])

                # Set up problem stage self.M
                if not determine_w:
                    W = self.R_mhe_all[i, :, :]
                    self.solver.cost_set(self.M, "W", W, api="new")
                    self.solver.cost_set(
                        self.M, "yref", yref[-self.n_measurement_noises :]
                    )
                    self.solver.cost_set(self.M, "scaling", self.cost_scaling)
                p = np.concatenate(
                    (
                        self.inputs_int[:, t],
                        self.outputs_int[:, t],
                    )
                )
                self.solver.set(self.M, "p", p)
                self.solver.set(self.M, "x", x_warmstart[i, t - self.M, :, self.M])

                # Run solver
                status = self.solver.solve()
                if status != 0:
                    print(
                        f"Solver for estimating state at t={t} (index {t - self.M}) returned status {status}: {helpers.get_acados_status_message(status)}"
                    )
                    # raise Exception(
                    #     f"Solver for estimating state at t={t} (index {t - self.M}) returned status {status}: {helpers.get_acados_status_message(status)}"
                    # )

                # Store result:
                # - estimated state of the current timestep
                # - estimated disturbances of the current timestep
                # - estimated measurement noises of the current timestep
                # - total cost
                # - terminal cost
                # - ground truth cost (if available)
                for k in range(self.M):
                    self.x_mhe_all[i, t - self.M, :, k] = self.solver.get(k, "x")
                    self.w_mhe_all[i, t - self.M, :, k] = self.solver.get(k, "u")
                    self.eta_mhe_all[i, t - self.M, :, k] = self.F_transpose @ (
                        self.outputs_int[:, t - self.M + k]
                        - self.model.get_outputs(
                            self.x_mhe_all[i, t - self.M, self.output_idc, k],
                            self.inputs_int[:, t - self.M + k],
                        )
                    )
                self.x_mhe_all[i, t - self.M, :, self.M] = self.solver.get(self.M, "x")
                self.eta_mhe_all[i, t - self.M, :, self.M] = self.F_transpose @ (
                    self.outputs_int[:, t]
                    - self.model.get_outputs(
                        self.x_mhe_all[i, t - self.M, self.output_idc, self.M],
                        self.inputs_int[:, t],
                    )
                )
                self.costs_total[i, t - self.M] = self.solver.get_cost()
                self.costs_term[i, t - self.M] = helpers.get_cost_terminal(
                    self.eta_mhe_all[i, t - self.M, :, self.M].reshape(
                        self.n_measurement_noises, 1
                    ),
                    self.R_mhe_all[i, :, :],
                )
                self.costs_w[i, t - self.M] = helpers.get_cost_mhe(
                    self.M,
                    self.w_mhe_all[i, t - self.M, -self.n_disturbances :, :],
                    np.zeros(
                        (self.n_measurement_noises, self.M + 1)
                    ),  # no measurement noises in this case
                    self.Q_mhe_all[i, :, :],
                    self.R_mhe_all[i, :, :],
                    np.arange(self.M + 1),
                )
                self.costs_eta[i, t - self.M] = helpers.get_cost_mhe(
                    self.M,
                    np.zeros(
                        (self.n_disturbances, self.M)
                    ),  # no disturbances in this case
                    self.eta_mhe_all[i, t - self.M, :, :],
                    self.Q_mhe_all[i, :, :],
                    self.R_mhe_all[i, :, :],
                    np.arange(self.M + 1),
                )
                if self.disturbances_gt_known:
                    self.costs_w_gt[i, t - self.M] = helpers.get_cost_mhe(
                        self.M,
                        self.disturbances_int[-self.n_disturbances :, t - self.M : t],
                        np.zeros(
                            (self.n_measurement_noises, self.M + 1)
                        ),  # no measurement noises in this case
                        self.Q_mhe_all[i, :, :],
                        self.R_mhe_all[i, :, :],
                        np.arange(self.M + 1),
                    )
                if self.measurement_noises_gt_known:
                    self.costs_eta_gt[i, t - self.M] = helpers.get_cost_mhe(
                        self.M,
                        np.zeros(
                            (self.n_disturbances, self.M)
                        ),  # no disturbances in this case
                        self.measurement_noises_int[:, t - self.M : t + 1],
                        self.Q_mhe_all[i, :, :],
                        self.R_mhe_all[i, :, :],
                        np.arange(self.M + 1),
                    )
                if self.disturbances_gt_known and self.measurement_noises_gt_known:
                    self.costs_total_gt[i, t - self.M] = helpers.get_cost_mhe(
                        self.M,
                        self.disturbances_int[-self.n_disturbances :, t - self.M : t],
                        self.measurement_noises_int[:, t - self.M : t + 1],
                        self.Q_mhe_all[i, :, :],
                        self.R_mhe_all[i, :, :],
                        np.arange(self.M + 1),
                    )

                # Create warm-start for run at next timestep: current optimal solution + one forward-simulated step
                if t < self.mhe_n_times - 1:
                    for k in range(self.M):
                        x_warmstart[i, t - self.M + 1, :, k] = self.x_mhe_all[
                            i, t - self.M, :, k + 1
                        ]
                        if k < self.M - 1:
                            u_warmstart[i, t - self.M + 1, :, k] = self.w_mhe_all[
                                i, t - self.M, :, k + 1
                            ]
                        else:
                            if self.determine_w:
                                u_warmstart[i, t - self.M + 1, :, k] = np.zeros(
                                    (self.n_states,)
                                )
                            else:
                                u_warmstart[i, t - self.M + 1, :, k] = np.zeros(
                                    (self.n_disturbances,)
                                )
                    x_warmstart[i, t - self.M + 1, :, self.M] = np.array(
                        helpers.solve_rk4(
                            self.model.state_update_ct,
                            x_warmstart[i, t - self.M + 1, :, self.M - 1],
                            self.inputs_int[:, t],
                            self.ts,
                        )
                    ).flatten()
                # Copy the first solution to the first warm-start in the next iteration (if there is one)
                elif i < self.mhe_n_iter - 1:
                    x_warmstart[i + 1, 0, :, :] = self.x_mhe_all[i, 0, :, :]
                    u_warmstart[i + 1, 0, :, :] = self.w_mhe_all[i, 0, :, :]

            # Update Q_cov and R_cov based on estimated disturbances measurement noises
            # Estimate full covariance matrices Q and R
            Q_est = np.cov(
                np.squeeze(self.w_mhe_all[i, :, :, self.stage_est]), rowvar=False
            )
            R_est = np.cov(
                np.squeeze(self.eta_mhe_all[i, :, :, self.stage_est]), rowvar=False
            )

            # Update covariance matrices Q and R if desired
            if not determine_w:
                if self.update_Q:
                    self.Q_cov_est_all[i + 1, :, :] = Q_est
                else:
                    self.Q_cov_est_all[i + 1, :, :] = self.Q_cov_est_all[i, :, :]
                if self.update_R:
                    self.R_cov_est_all[i + 1, :, :] = R_est
                else:
                    self.R_cov_est_all[i + 1, :, :] = self.R_cov_est_all[i, :, :]

            # Compute disturbance values
            if self.determine_w:
                self.w = np.zeros((self.n_states, self.mhe_n_times - 1))
                for idx in range(self.mhe_n_times - self.M):
                    if idx == 0:
                        self.w[:, : self.stage_est + 1] = self.w_mhe_all[
                            0, idx, :, : self.stage_est + 1
                        ]
                    elif idx == self.mhe_n_times - self.M - 1:
                        self.w[:, idx + self.stage_est :] = self.w_mhe_all[
                            0, idx, :, self.stage_est :
                        ]
                    else:
                        self.w[:, idx + self.stage_est] = self.w_mhe_all[
                            0, idx, :, self.stage_est
                        ]

            # Compute and print model mismatch bounds resulting from this iteration
            self.compute_model_mismatch_bounds(i)

        # Store disturbance data in json file
        if self.determine_w:
            data_w = {
                "t": (
                    self.times_max_begin + self.times_int[: self.mhe_n_times - 1]
                ).tolist(),
                "u": self.inputs_int[:, : self.mhe_n_times - 1].tolist(),
                "y": self.outputs_int[:, : self.mhe_n_times - 1].tolist(),
                "w": self.w.tolist(),
                "w_mhe_all": self.w_mhe_all[-1, :, :, :].tolist(),
            }
            with open(self.w_json_path, "w") as f:
                json.dump(
                    data_w,
                    f,
                )
            print(f"\nDisturbance data stored in {self.w_json_path}")

    def compute_model_mismatch_bounds(self, iter_idx):
        # Compute ground truth disturbance and measurement noise bounds
        if self.disturbances_gt_known:
            self.disturbances_min_gt_abs = np.min(self.disturbances_int, axis=1)
            self.disturbances_max_gt_abs = np.max(self.disturbances_int, axis=1)
            self.disturbances_bias_gt = (
                self.disturbances_max_gt_abs + self.disturbances_min_gt_abs
            ) / 2
            self.disturbances_min_gt_rel = (
                self.disturbances_min_gt_abs - self.disturbances_bias_gt
            )
            self.disturbances_max_gt_rel = (
                self.disturbances_max_gt_abs - self.disturbances_bias_gt
            )
        if self.measurement_noises_gt_known:
            self.measurement_noises_min_gt_abs = np.min(
                self.measurement_noises_int, axis=1
            )
            self.measurement_noises_max_gt_abs = np.max(
                self.measurement_noises_int, axis=1
            )

        # Compute estimated disturbance and measurement noise bounds
        if self.determine_w:
            self.disturbances_min_est_abs = np.min(self.w, axis=1)
            self.disturbances_max_est_abs = np.max(self.w, axis=1)
        else:
            self.disturbances_min_est_abs = np.min(
                self.w_mhe_all[iter_idx, :, :, self.stage_est], axis=0
            )
            self.disturbances_max_est_abs = np.max(
                self.w_mhe_all[iter_idx, :, :, self.stage_est], axis=0
            )
            # self.disturbances_min_est_abs = np.min(self.w_mhe_all[-1, :, :, :], axis=(0, 2))
            # self.disturbances_max_est_abs = np.max(self.w_mhe_all[-1, :, :, :], axis=(0, 2))
            self.disturbances_bias_est = (
                self.disturbances_max_est_abs + self.disturbances_min_est_abs
            ) / 2
            self.disturbances_min_est_rel = (
                self.disturbances_min_est_abs - self.disturbances_bias_est
            )
            self.disturbances_max_est_rel = (
                self.disturbances_max_est_abs - self.disturbances_bias_est
            )
            if self.disturbances_gt_known:
                self.disturbances_min_abs_rmse = mean_squared_error(
                    self.disturbances_min_gt_abs,
                    self.disturbances_min_est_abs,
                    squared=False,
                )
                self.disturbances_max_abs_rmse = mean_squared_error(
                    self.disturbances_max_gt_abs,
                    self.disturbances_max_est_abs,
                    squared=False,
                )
                self.disturbances_total_abs_rmse = mean_squared_error(
                    np.concatenate(
                        [
                            self.disturbances_min_gt_abs,
                            self.disturbances_max_gt_abs,
                        ]
                    ),
                    np.concatenate(
                        [
                            self.disturbances_min_est_abs,
                            self.disturbances_max_est_abs,
                        ]
                    ),
                    squared=False,
                )
            else:
                log.warning(
                    "Ground truth disturbances are not known. Cannot compute RMSE for lower, upper, and total disturbance bounds. Setting printing to False"
                )
                self.do_print_disturbances_min_rmse = False
                self.do_print_disturbances_max_rmse = False
                self.do_print_disturbances_total_rmse = False
        self.meas_noises_min_est_abs = np.min(
            self.eta_mhe_all[iter_idx, :, :, self.stage_est], axis=0
        )
        self.meas_noises_max_est_abs = np.max(
            self.eta_mhe_all[iter_idx, :, :, self.stage_est], axis=0
        )
        # self.meas_noises_min_est_abs = np.min(self.eta_mhe_all[-1, :, :, :], axis=(0, 2))
        # self.meas_noises_max_est_abs = np.max(self.eta_mhe_all[-1, :, :, :], axis=(0, 2))

        # Print results if desired
        do_print_disturbances = (
            self.do_print_disturbances_min
            or self.do_print_disturbances_max
            or self.do_print_disturbances_bias
            or self.do_print_disturbances_min_rmse
            or self.do_print_disturbances_max_rmse
            or self.do_print_disturbances_total_rmse
        )
        if do_print_disturbances:
            print(f"\nDisturbance bounds:")
            if self.do_print_disturbances_min:
                print(f"Min:     {self.disturbances_min_est_abs}")
                # print(f"Min:     {self.disturbances_min_est_rel}")
                if self.disturbances_gt_known:
                    print(
                        f"Min GT:  {self.disturbances_min_gt_abs[-self.n_disturbances :]}"
                    )
                    if np.all(
                        self.disturbances_min_gt_abs[-self.n_disturbances :] != 0
                    ):
                        print(
                            f"Min ratio:  {self.disturbances_min_est_abs / self.disturbances_min_gt_abs[-self.n_disturbances :]}"
                        )
            if self.do_print_disturbances_max:
                print(f"Max:     {self.disturbances_max_est_abs}")
                # print(f"Max:     {self.disturbances_max_est_rel}")
                if self.disturbances_gt_known:
                    print(
                        f"Max GT:  {self.disturbances_max_gt_abs[-self.n_disturbances :]}"
                    )
                    if np.all(
                        self.disturbances_max_gt_abs[-self.n_disturbances :] != 0
                    ):
                        print(
                            f"Max ratio:  {self.disturbances_max_est_abs / self.disturbances_max_gt_abs[-self.n_disturbances :]}"
                        )
            if not self.determine_w:
                if self.do_print_disturbances_bias:
                    print(f"Bias:    {self.disturbances_bias_est}")
                    if self.disturbances_gt_known:
                        print(
                            f"Bias GT: {self.disturbances_bias_gt[-self.n_disturbances :]}"
                        )
            if not self.determine_w and self.do_print_disturbances_min_rmse:
                print(f"Min RMSE: {self.disturbances_min_abs_rmse}")
            if not self.determine_w and self.do_print_disturbances_max_rmse:
                print(f"Max RMSE: {self.disturbances_max_abs_rmse}")
            if not self.determine_w and self.do_print_disturbances_total_rmse:
                print(f"Total RMSE: {self.disturbances_total_abs_rmse}")

        do_print_meas_noises = (
            self.do_print_meas_noises_min or self.do_print_meas_noises_max
        )
        if do_print_meas_noises:
            print(f"\nMeasurement noise bounds:")
            if self.do_print_meas_noises_min:
                print(f"Min:     {self.meas_noises_min_est_abs}")
                if self.measurement_noises_gt_known:
                    print(f"Min GT:  {self.measurement_noises_min_gt_abs}")
                    if np.all(self.measurement_noises_min_gt_abs != 0):
                        print(
                            f"Min ratio:  {self.meas_noises_min_est_abs / self.measurement_noises_min_gt_abs}"
                        )
            if self.do_print_meas_noises_max:
                print(f"Max:     {self.meas_noises_max_est_abs}")
                if self.measurement_noises_gt_known:
                    print(f"Max GT:  {self.measurement_noises_max_gt_abs}")
                    if np.all(self.measurement_noises_max_gt_abs != 0):
                        print(
                            f"Max ratio:  {self.meas_noises_max_est_abs / self.measurement_noises_max_gt_abs}"
                        )

    def get_json_specific_data(self):
        data_general = {
            "t": self.times_int[: self.mhe_n_times].tolist(),
            "y": self.outputs_int[:, : self.mhe_n_times].tolist(),
            "u": self.inputs_int[:, : self.mhe_n_times].tolist(),
        }
        data_gt = {}
        if self.disturbances_gt_known:
            data_gt["w"] = self.disturbances_int[:, : self.mhe_n_times].tolist()
            data_gt["w_min"] = self.disturbances_min_gt_abs.tolist()
            data_gt["w_max"] = self.disturbances_max_gt_abs.tolist()
            data_gt["costs_w_gt"] = self.costs_w_gt.tolist()
        if self.measurement_noises_gt_known:
            data_gt["eta"] = self.measurement_noises_int[:, : self.mhe_n_times].tolist()
            data_gt["eta_min"] = self.measurement_noises_min_gt_abs.tolist()
            data_gt["eta_max"] = self.measurement_noises_max_gt_abs.tolist()
            data_gt["costs_eta_gt"] = self.costs_eta_gt.tolist()
        if self.disturbances_gt_known and self.measurement_noises_gt_known:
            data_gt["total_gt_costs"] = self.costs_total_gt.tolist()
        data_mhe = {
            "Q_cov_est_all": self.Q_cov_est_all.tolist(),
            "R_cov_est_all": self.R_cov_est_all.tolist(),
            "eps": self.eps,
            "Q_mhe_all": self.Q_mhe_all.tolist(),
            "R_mhe_all": self.R_mhe_all.tolist(),
            "x_est_all": self.x_mhe_all.tolist(),
            "w_est_all": self.w_mhe_all.tolist(),
            "eta_est_all": self.eta_mhe_all.tolist(),
        }
        data_costs = {}
        data_costs["costs_total"] = self.costs_total.tolist()
        data_costs["costs_term"] = self.costs_term.tolist()
        data_costs["costs_w"] = self.costs_w.tolist()
        data_costs["costs_eta"] = self.costs_eta.tolist()
        if self.disturbances_gt_known:
            data_costs["costs_w_gt"] = self.costs_w_gt.tolist()
        if self.measurement_noises_gt_known:
            data_costs["costs_eta_gt"] = self.costs_eta_gt.tolist()
        if self.disturbances_gt_known and self.measurement_noises_gt_known:
            data_costs["costs_total_gt"] = self.costs_total_gt.tolist()
        data_stat = {
            "w_min_est_abs": self.disturbances_min_est_abs.tolist(),
            "w_max_est_abs": self.disturbances_max_est_abs.tolist(),
            "eta_min_est_abs": self.meas_noises_min_est_abs.tolist(),
            "eta_max_est_abs": self.meas_noises_max_est_abs.tolist(),
        }
        data_rmse = {}
        if not self.determine_w:
            data_rmse["w_min_abs_rmse"] = self.disturbances_min_abs_rmse
            data_rmse["w_max_abs_rmse"] = self.disturbances_max_abs_rmse
            data_rmse["w_total_abs_rmse"] = self.disturbances_total_abs_rmse
        data = {
            k: v
            for d in (data_general, data_gt, data_mhe, data_costs, data_stat, data_rmse)
            for k, v in d.items()
        }
        return data

    def plot_raw_interp_inputs(self):
        inputs_idc = np.where(
            np.logical_and(
                self.inputs_times >= self.times_max_begin,
                self.inputs_times <= self.times_min_end,
            )
        )[0]
        self.inputs_times = self.inputs_times[inputs_idc]
        self.inputs = self.inputs[:, inputs_idc]
        self.inputs_times = self.inputs_times - self.times_max_begin

        handles_raw = []
        handles_int = []
        fig, axes = plt.subplots(
            self.n_rows_inputs, self.n_cols_inputs, num="Raw vs interpolated inputs"
        )
        fig.suptitle("Raw vs interpolated inputs")
        for input_idx in range(self.n_inputs):
            row_idx = input_idx // self.n_cols_inputs
            col_idx = input_idx % self.n_cols_inputs
            handles_raw.append(
                axes[row_idx, col_idx].plot(
                    self.inputs_times,
                    self.inputs[input_idx, :],
                    "--o",
                    linewidth=self.widths,
                    markersize=self.sizes,
                    label="Raw",
                )
            )
            handles_int.append(
                axes[row_idx, col_idx].plot(
                    self.times_int,
                    self.inputs_int[input_idx, :],
                    "o",
                    markersize=self.sizes,
                    label="Interpolated",
                )
            )
            axes[row_idx, col_idx].set_xlabel("Time (s)")
            axes[row_idx, col_idx].set_ylabel(self.u_labels[input_idx])
        fig.legend([handles_raw[0][0].get_label(), handles_int[0][0].get_label()])

    def plot_raw_interp_outputs(self):
        handles_raw = []
        handles_int = []
        fig, axes = plt.subplots(
            self.n_rows_states, self.n_cols_states, num="Raw vs interpolated outputs"
        )
        fig.suptitle("Raw vs interpolated outputs")
        outputs_idc = np.where(
            np.logical_and(
                self.outputs_times >= self.times_max_begin,
                self.outputs_times <= self.times_min_end,
            )
        )[0]
        self.outputs_times = self.outputs_times[outputs_idc]
        self.outputs_times = self.outputs_times - self.times_max_begin
        self.outputs = self.outputs[:, outputs_idc]

        for ax_idx in range(self.n_rows_states * self.n_cols_states):
            if self.plot_y_idx_at_ax_idx[ax_idx] == None:
                axes.flat[ax_idx].axis("off")
                continue
            row_idx = ax_idx // self.n_cols_states
            col_idx = ax_idx % self.n_cols_states
            y_idx = self.plot_y_idx_at_ax_idx[ax_idx]
            handles_raw.append(
                axes[row_idx, col_idx].plot(
                    self.outputs_times,
                    self.outputs[y_idx, :],
                    "--o",
                    linewidth=self.widths,
                    markersize=self.sizes,
                    label="Raw",
                )
            )
            handles_int.append(
                axes[row_idx, col_idx].plot(
                    self.times_int,
                    self.outputs_int[y_idx, :],
                    "o",
                    markersize=self.sizes,
                    label="Interpolated",
                )
            )
            axes[row_idx, col_idx].set_xlabel("Time (s)")
            axes[row_idx, col_idx].set_ylabel(self.y_labels[y_idx])
        fig.legend([handles_raw[0][0].get_label(), handles_int[0][0].get_label()])

    def plot_raw_interp_disturbances(self):
        handles_raw = []
        handles_int = []
        fig, axes = plt.subplots(
            self.n_rows_states,
            self.n_cols_states,
            num="Raw vs interpolated disturbances",
        )
        fig.suptitle("Raw vs interpolated disturbances")
        disturbances_idc = np.where(
            np.logical_and(
                self.disturbances_times >= self.times_max_begin,
                self.disturbances_times <= self.times_min_end,
            )
        )[0]
        self.disturbances_times = self.disturbances_times[disturbances_idc]
        self.disturbances_times = self.disturbances_times - self.times_max_begin
        self.disturbances = self.disturbances[:, disturbances_idc]

        for ax_idx in range(self.n_rows_states * self.n_cols_states):
            if self.plot_w_idx_at_ax_idx[ax_idx] == None:
                axes.flat[ax_idx].axis("off")
                continue
            row_idx = ax_idx // self.n_cols_states
            col_idx = ax_idx % self.n_cols_states
            w_idx = self.plot_w_idx_at_ax_idx[ax_idx]
            handles_raw.append(
                axes[row_idx, col_idx].plot(
                    self.disturbances_times,
                    self.disturbances[w_idx, :],
                    "--o",
                    linewidth=self.widths,
                    markersize=self.sizes,
                    label="Raw",
                )
            )
            handles_int.append(
                axes[row_idx, col_idx].plot(
                    self.times_int,
                    self.disturbances_int[w_idx, :],
                    "o",
                    markersize=self.sizes,
                    label="Interpolated",
                )
            )
            axes[row_idx, col_idx].set_xlabel("Time (s)")
            axes[row_idx, col_idx].set_ylabel(self.w_labels[w_idx])
        fig.legend([handles_raw[0][0].get_label(), handles_int[0][0].get_label()])

    def plot_raw_interp_meas_noises(self):
        handles_raw = []
        handles_int = []
        fig, axes = plt.subplots(
            self.n_rows_states,
            self.n_cols_states,
            num="Raw vs interpolated measurement noises",
        )
        fig.suptitle("Raw vs interpolated measurement noises")
        meas_noises_idc = np.where(
            np.logical_and(
                self.measurement_noises_times >= self.times_max_begin,
                self.measurement_noises_times <= self.times_min_end,
            )
        )[0]
        self.measurement_noises_times = self.measurement_noises_times[meas_noises_idc]
        self.measurement_noises_times = (
            self.measurement_noises_times - self.times_max_begin
        )
        self.measurement_noises = self.measurement_noises[:, meas_noises_idc]
        for ax_idx in range(self.n_rows_states * self.n_cols_states):
            if self.plot_eta_idx_at_ax_idx[ax_idx] == None:
                axes.flat[ax_idx].axis("off")
                continue
            row_idx = ax_idx // self.n_cols_states
            col_idx = ax_idx % self.n_cols_states
            eta_idx = self.plot_eta_idx_at_ax_idx[ax_idx]
            handles_raw.append(
                axes[row_idx, col_idx].plot(
                    self.measurement_noises_times,
                    self.measurement_noises[eta_idx, :],
                    "--o",
                    linewidth=self.widths,
                    markersize=self.sizes,
                    label="Raw",
                )
            )
            handles_int.append(
                axes[row_idx, col_idx].plot(
                    self.times_int,
                    self.measurement_noises_int[eta_idx, :],
                    "o",
                    markersize=self.sizes,
                    label="Interpolated",
                )
            )
            axes[row_idx, col_idx].set_xlabel("Time (s)")
            axes[row_idx, col_idx].set_ylabel(self.eta_labels[eta_idx])
        fig.legend([handles_raw[0][0].get_label(), handles_int[0][0].get_label()])

    def create_plots(self):
        # General plot settings
        self.sizes = 2
        self.widths = 0.5
        self.n_rows_inputs = 2
        self.n_cols_inputs = 2
        self.n_rows_states = 4
        # self.n_rows_states = 5
        self.n_cols_states = 4
        self.plot_y_idx_at_ax_idx = [
            0,
            1,
            2,
            None,
            3,
            4,
            5,
            None,
            6,
            7,
            8,
            None,
            9,
            10,
            11,
            None,
        ]
        self.plot_w_idx_at_ax_idx = [
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            0,
            1,
            2,
            None,
            3,
            4,
            5,
            None,
        ]
        self.plot_eta_idx_at_ax_idx = [
            0,
            1,
            2,
            None,
            3,
            4,
            5,
            None,
            6,
            7,
            8,
            None,
            9,
            10,
            11,
            None,
        ]
        self.y_labels = [
            "px (m)",
            "py (m)",
            "pz (m)",
            "$\phi$ (rad)",
            "$\\theta$ (rad)",
            "$\psi$ (rad)",
            "vx (m/s)",
            "vy (m/s)",
            "vz (m/s)",
            "wbx (rad/s)",
            "wby (rad/s)",
            "wbz (rad/s)",
        ]
        self.u_labels = [
            "t0c (rad/s)",
            "t1c (rad/s)",
            "t2c (rad/s)",
            "t3c (m/s^2)",
        ]
        self.w_labels = [
            "$w_{vx} (m/s / s)$",
            "$w_{vy} (m/s / s)$",
            "$w_{vz} (m/s / s)$",
            "$w_{wbx} (rad/s / s)$",
            "$w_{wby} (rad/s / s)$",
            "$w_{wbz} (rad/s / s)$",
        ]
        self.eta_labels = self.y_labels

        # Plot raw vs interpolated data
        if self.do_plot_raw_interp_inputs:
            self.plot_raw_interp_inputs()
        if self.do_plot_raw_interp_outputs:
            self.plot_raw_interp_outputs()
        if self.do_plot_raw_interp_disturbances:
            self.plot_raw_interp_disturbances()
        if self.do_plot_raw_interp_meas_noises:
            self.plot_raw_interp_meas_noises()


def compute_absolute_w_eta_bounds(data, determine_w, sim_eta_max, do_print_w_eta):
    # Store all absolute estimated disturbance and measurement noise bounds
    if determine_w:
        nw = data["common"]["nx"]
    else:
        nw = data["common"]["nw"]
    w_min_all = np.zeros((len(data) - 1, nw))
    w_max_all = np.zeros((len(data) - 1, nw))
    eta_min_all = np.zeros((len(data) - 1, data["common"]["neta"]))
    eta_max_all = np.zeros((len(data) - 1, data["common"]["neta"]))
    idx = 0
    for key in data.keys():
        if key == "common":
            continue
        w_min_all[idx, :] = data[key]["w_min_est_abs"]
        w_max_all[idx, :] = data[key]["w_max_est_abs"]
        eta_min_all[idx, :] = data[key]["eta_min_est_abs"]
        eta_max_all[idx, :] = data[key]["eta_max_est_abs"]
        idx += 1
    data["common"]["w_min_all"] = w_min_all.tolist()
    data["common"]["w_max_all"] = w_max_all.tolist()
    data["common"]["eta_min_all"] = eta_min_all.tolist()
    data["common"]["eta_max_all"] = eta_max_all.tolist()

    # Compute and store the overall absolute disturbance and measurement noise bounds
    w_min_abs = np.min(w_min_all, axis=0)
    w_max_abs = np.max(w_max_all, axis=0)
    eta_min_abs = np.min(eta_min_all, axis=0)
    eta_max_abs = np.max(eta_max_all, axis=0)
    data["common"]["w_min_abs"] = w_min_abs.tolist()
    data["common"]["w_max_abs"] = w_max_abs.tolist()
    data["common"]["eta_min_abs"] = eta_min_abs.tolist()
    data["common"]["eta_max_abs"] = eta_max_abs.tolist()

    # Compute and store the overall biases and corresponding relative disturbance and measurement noise bounds
    w_bias = (w_min_abs + w_max_abs) / 2
    w_min_rel = w_min_abs - w_bias
    w_max_rel = w_max_abs - w_bias
    eta_bias = (eta_min_abs + eta_max_abs) / 2
    eta_min_rel = eta_min_abs - eta_bias
    eta_max_rel = eta_max_abs - eta_bias
    data["common"]["w_bias"] = w_bias.tolist()
    data["common"]["w_min_rel"] = w_min_rel.tolist()
    data["common"]["w_max_rel"] = w_max_rel.tolist()
    data["common"]["eta_bias"] = eta_bias.tolist()
    data["common"]["eta_min_rel"] = eta_min_rel.tolist()
    data["common"]["eta_max_rel"] = eta_max_rel.tolist()

    # Compute and store the ground truth disturbance noise bounds
    disturbances_gt_known = True
    for key in data.keys():
        if key == "common":
            continue
        if "w" not in data[key]:
            log.warning(
                f"Key {key} does not contain ground truth disturbance data. Skipping storing and printing this data"
            )
            disturbances_gt_known = False
            break

    if disturbances_gt_known:
        w_min_all_gt = np.zeros((len(data) - 1, nw))
        w_max_all_gt = np.zeros((len(data) - 1, nw))
        idx = 0
        for key in data.keys():
            if key == "common":
                continue
            if "w" in data[key]:
                w_min_all_gt[idx, :] = data[key]["w_min"]
                w_max_all_gt[idx, :] = data[key]["w_max"]
            else:
                log.warning(
                    f"Key {key} does not contain ground truth disturbance data. Setting to zero"
                )
                w_min_all_gt[idx, :] = np.zeros(nw)
                w_max_all_gt[idx, :] = np.zeros(nw)
            idx += 1
        data["common"]["w_min_all_gt"] = w_min_all_gt.tolist()
        data["common"]["w_max_all_gt"] = w_max_all_gt.tolist()
        w_min_gt = np.min(w_min_all_gt, axis=0)
        w_max_gt = np.max(w_max_all_gt, axis=0)
        data["common"]["w_min_gt"] = w_min_gt.tolist()
        data["common"]["w_max_gt"] = w_max_gt.tolist()
        data["common"]["w_min_abs_rmse"] = mean_squared_error(
            w_min_gt, w_min_abs, squared=False
        )
        data["common"]["w_max_abs_rmse"] = mean_squared_error(
            w_max_gt, w_max_abs, squared=False
        )
        data["common"]["w_total_abs_rmse"] = mean_squared_error(
            np.concatenate([w_min_gt, w_max_gt]),
            np.concatenate([w_min_abs, w_max_abs]),
            squared=False,
        )
    else:
        log.warning(
            "Ground truth disturbances are not known. Cannot compute RMSE for overall lower, upper, and total disturbance bounds. Setting printing to False"
        )
        do_print_disturbances_min_rmse = False
        do_print_disturbances_max_rmse = False
        do_print_disturbances_total_rmse = False

    # Compute and store the ground truth measurement noise bounds
    sim_eta_min = -sim_eta_max
    data["common"]["eta_min_abs_gt"] = sim_eta_min.tolist()
    data["common"]["eta_max_abs_gt"] = sim_eta_max.tolist()

    # Print overall relative disturbance and measurement noise bounds
    do_print_disturbances_min = do_print_w_eta[0]
    do_print_disturbances_max = do_print_w_eta[1]
    do_print_disturbances_bias = do_print_w_eta[2]
    do_print_disturbances_min_rmse = do_print_w_eta[3]
    do_print_disturbances_max_rmse = do_print_w_eta[4]
    do_print_disturbances_total_rmse = do_print_w_eta[5]
    do_print_meas_noises_min = do_print_w_eta[6]
    do_print_meas_noises_max = do_print_w_eta[7]
    do_print_disturbances = (
        do_print_disturbances_min
        or do_print_disturbances_max
        or do_print_disturbances_bias
        or do_print_disturbances_min_rmse
        or do_print_disturbances_max_rmse
        or do_print_disturbances_total_rmse
    )
    do_print_meas_noises = do_print_meas_noises_min or do_print_meas_noises_max
    if do_print_disturbances:
        print(f"\nOverall disturbance bounds:")
        if do_print_disturbances_min:
            print(f'Min:     {data["common"]["w_min_abs"]}')
            if disturbances_gt_known:
                print(f'Min GT:  {data["common"]["w_min_gt"]}')
                if np.all(data["common"]["w_min_gt"] != 0):
                    print(
                        f'Min ratio:  {np.array(data["common"]["w_min_abs"]) / np.array(data["common"]["w_min_gt"])}'
                    )
        if do_print_disturbances_max:
            print(f'Max:     {data["common"]["w_max_abs"]}')
            if disturbances_gt_known:
                print(f'Max GT:  {data["common"]["w_max_gt"]}')
                if np.all(data["common"]["w_max_gt"] != 0):
                    print(
                        f'Max ratio:  {np.array(data["common"]["w_max_abs"]) / np.array(data["common"]["w_max_gt"])}'
                    )
        # if do_print_disturbances_bias:
        #     print(f'Bias:    {data["common"]["w_bias"]}')
        if not determine_w and do_print_disturbances_min_rmse:
            print(f'Min RMSE: {data["common"]["w_min_abs_rmse"]}')
        if not determine_w and do_print_disturbances_max_rmse:
            print(f'Max RMSE: {data["common"]["w_max_abs_rmse"]}')
        if not determine_w and do_print_disturbances_total_rmse:
            print(f'Total RMSE: {data["common"]["w_total_abs_rmse"]}')
    if do_print_meas_noises:
        print(f"\nOverall measurement noise bounds:")
        if do_print_meas_noises_min:
            print(f'Min:     {data["common"]["eta_min_abs"]}')
        if do_print_meas_noises_max:
            print(f'Max:     {data["common"]["eta_max_abs"]}')


if __name__ == "__main__":
    # Start timing
    start = time.time()

    # Log settings
    parser = argparse.ArgumentParser(description="something")
    parser.add_argument("-v", "--verbose", action="count", default=0, dest="verbosity")
    args = parser.parse_args()
    logging.basicConfig()
    logging.getLogger().setLevel(logging.WARN - 10 * args.verbosity)

    # Print settings
    np.set_printoptions(linewidth=np.inf, precision=10)

    # User settings
    package_dir = Path(__file__).parents[1]
    json_dir = f"{package_dir}/../rosbag2json/data/converted_bags"
    if not path.exists(json_dir):
        log.warning(
            f"Directory {json_dir} does not exist! Please ensure that the rosbag2json submodule is cloned"
        )
        exit(1)
    config_dir = f"{package_dir}/config"
    config_path = f"{config_dir}/scripts/determine_model_mismatch.yaml"
    data_dir = f"{package_dir}/data"
    data_sel_dir = f"{data_dir}/selected_data"
    data_sel_file_name = "model_mismatch_data_select.json"
    output_data_dir = f"{data_dir}/model_mismatch_results"

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    # Get simulation noise settings
    sim_w_max = np.array(config["simulation"]["w_max"])
    sim_eta_max = np.array(config["simulation"]["eta_max"])
    sim_eta_max_scaling = config["simulation"]["eta_max_scaling"]

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

    # Get json file names
    json_names = config["recorded_data"]["json_names"]

    # Get sampling time
    ts = config["recorded_data"]["processing"]["ts"]

    # Get MHE solver parameters
    generate_solver = config["mhe"]["generate_solver"]
    M = config["mhe"]["M"]
    stage_est = config["mhe"]["stage_est"]
    eps = float(config["mhe"]["eps"])
    cost_scaling = float(config["mhe"]["cost_scaling"])
    determine_w = config["mhe"]["determine_w"]
    use_predetermined_w = config["mhe"]["use_predetermined_w"]

    # Handle settings related to determining w
    if determine_w:
        if use_predetermined_w:
            log.warning(
                f"The 'use_predetermined_w' setting cannot be combined with 'determine_w'. It will be set to False"
            )
            use_predetermined_w = False
        if any("sim" in json_name for json_name in json_names):
            log.warning(
                f"The 'determine_w' setting is only relevant for the Gazebo experiment type. Detected at least one sim experiment, so it will be set to False"
            )
            determine_w = False

    if use_predetermined_w:
        if any("sim" in json_name for json_name in json_names):
            log.warning(
                f"The 'use_predetermined_w' setting is only relevant for the Gazebo experiment type. Detected at least one sim experiment, so it will be set to False"
            )
            use_predetermined_w = False

    # Get printing options
    do_print_disturbances_min = config["printing"]["disturbances"]["min"]
    do_print_disturbances_max = config["printing"]["disturbances"]["max"]
    do_print_disturbances_bias = config["printing"]["disturbances"]["bias"]
    do_print_disturbances_min_rmse = config["printing"]["disturbances"]["min_rmse"]
    do_print_disturbances_max_rmse = config["printing"]["disturbances"]["max_rmse"]
    do_print_disturbances_total_rmse = config["printing"]["disturbances"]["total_rmse"]
    do_print_meas_noises_min = config["printing"]["meas_noises"]["min"]
    do_print_meas_noises_max = config["printing"]["meas_noises"]["max"]
    do_print_w_eta = [
        do_print_disturbances_min,
        do_print_disturbances_max,
        do_print_disturbances_bias,
        do_print_disturbances_min_rmse,
        do_print_disturbances_max_rmse,
        do_print_disturbances_total_rmse,
        do_print_meas_noises_min,
        do_print_meas_noises_max,
    ]

    # Create dictionary with common data, to be filled later with json-specific data
    data = {}
    model_name = model.get_name()
    data_general = {
        "g": g,
        "ts": ts,
        "params_file": params_file,
        "model_name": model_name,
        "M": M,
        "stage_est": stage_est,
        "eps": eps,
        "cost_scaling": cost_scaling,
    }
    data_model = model.get_model_data()
    data["common"] = {k: v for d in (data_general, data_model) for k, v in d.items()}

    # Generate MHE solver
    solver = helpers.get_acados_mhe_solver(
        model, M, ts, sim_eta_max, sim_eta_max_scaling, determine_w, generate_solver
    )

    # Process json data files
    for json_name in json_names:
        print()
        print("-" * 100)
        log.warning(f"Selected json file: {json_name}")

        # Process file name
        _, exp_type, _, _, exp_details = helpers.read_file_name(json_name[:-5])
        exp_details = "_".join([str(e) for e in exp_details])

        compute_model_mismatch = ComputeModelMismatch(
            config,
            json_dir,
            json_name,
            data_sel_dir,
            data_sel_file_name,
            output_data_dir,
            exp_type,
            model,
            determine_w,
            use_predetermined_w,
            solver,
            sim_w_max,
            sim_eta_max,
        )
        compute_model_mismatch.process_recorded_data()
        compute_model_mismatch.compute_model_mismatch_mhe()
        if not determine_w:
            compute_model_mismatch.create_plots()
        data[exp_details] = compute_model_mismatch.get_json_specific_data()
        print("-" * 100)

    compute_absolute_w_eta_bounds(data, determine_w, sim_eta_max, do_print_w_eta)

    if not determine_w:
        # Save data to json file for plotting
        output_data_json_path = f"{output_data_dir}/{model_name}.json"
        with open(output_data_json_path, "w") as json_file:
            json.dump(data, json_file, indent=4)
        print(f"\nSaved data to {output_data_json_path}")

        # Save data to mat file for SDP
        output_data_mat_path = f"{output_data_dir}/{model_name}.mat"
        scipy.io.savemat(output_data_mat_path, data["common"])
        print(f"Saved data to {output_data_mat_path}")

    # End timing and print
    end = time.time()
    print(f"\nElapsed time: {end - start}")

    # Show plots if indicated
    plt.show()
