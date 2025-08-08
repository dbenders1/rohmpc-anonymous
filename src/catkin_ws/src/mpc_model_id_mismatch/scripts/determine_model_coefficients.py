import argparse
import json
import logging
import time
import yaml

import casadi as ca
import matplotlib.pyplot as plt
import numpy as np
from os import path
import scipy.signal as signal

from mpc_model_id_mismatch import helpers
from numpy import linalg
from pathlib import Path
from scipy import interpolate

# Logging functionality
log = logging.getLogger(__name__)

np.set_printoptions(precision=10)


class DetermineModelCoefficients:
    def __init__(
        self,
        config,
        config_dir,
        json_dir,
        json_name,
        data_sel_dir,
        data_sel_file_name,
        exp_type,
    ) -> None:
        # Process config
        self.config = config

        self.g = self.config["constants"]["g"]

        self.overwrite_data_sel = config["recorded_data"]["overwrite_data_sel"]
        self.ts = self.config["recorded_data"]["processing"]["ts"]

        self.model_name = self.config["model"]["name"]

        # Process other arguments
        self.json_dir = json_dir
        self.json_name = json_name
        self.data_sel_dir = data_sel_dir
        self.data_sel_file_name = data_sel_file_name
        self.exp_type = exp_type

        # Check if quad param file exists
        params_file = f"{config_dir}/systems/{self.model_name}.yaml"
        if path.exists(params_file):
            log.warning(f"Selected {self.model_name} params file.")
        else:
            log.fatal(f"Unknown quad name {self.model_name}! Exiting.")
            exit(1)

        # Create model object
        if self.model_name == "falcon":
            self.model = helpers.DroneAgiModel(self.model_name, self.g, params_file)
        else:
            log.fatal(f"Unknown model {self.model_name}! Exiting.")
            exit(1)

        # Design low-pass filter
        # fc_lb = 2 / 0.033  # Nyquist frequency fastest dynamics
        # fc_ub = 1 / (2 * self.ts)  # Nyquist frequency sampling frequency
        # fc = 0.5 * (fc_lb + fc_ub)  # Cut-off frequency in Hz
        fc = 74  # Cut-off frequency in Hz
        wc = 2 * np.pi * fc  # rad/s
        num_lp = [wc]
        den = [1, wc]
        self.lp_filter = signal.TransferFunction(num_lp, den)

        # Design high-pass filter
        num_hp = [wc, 0]
        self.hp_filter = signal.TransferFunction(num_hp, den)

        # Define start time for estimation when using the filters above
        self.filters_start_time = 10

    def estimate_motortau(self):
        # Set dimensions
        x_wm_idx_start = 12
        n_wm = 4
        n_param = 1

        # Get dimensions
        n_x = self.model.get_n_states()
        n_w = self.model.get_n_disturbances()
        n_points_per_time = n_wm
        n_times = self.outputs_int.shape[1] - self.filters_start_time

        # Set motor velocity state indices
        x_wm_idc = np.linspace(x_wm_idx_start, x_wm_idx_start + n_wm - 1, n_wm).astype(
            int
        )

        # Compute states and inputs using the low-pass filter
        x_filt = np.array(
            [
                signal.lsim(self.lp_filter, U=row.T, T=self.times_int)[1]
                for row in self.outputs_int
            ]
        ).T
        x_filt = x_filt[self.filters_start_time :, :]
        u_filt = np.array(
            [
                signal.lsim(self.lp_filter, U=row.T, T=self.times_int)[1]
                for row in self.inputs_int
            ]
        ).T
        u_filt = u_filt[self.filters_start_time :, :]

        # Compute the derivative of the states using the high-pass filter
        dx_filt = np.array(
            [
                signal.lsim(self.hp_filter, U=row.T, T=self.times_int)[1]
                for row in self.outputs_int
            ]
        ).T
        dx_filt = dx_filt[self.filters_start_time :, :]

        # Create symbolic variables to define symbolic vectors and matrices
        x_sym = ca.MX.sym("x", n_x)
        u_sym = ca.MX.sym("u", self.model.get_n_inputs())
        theta_sym = ca.MX.sym("theta_sym", 1)

        # Compute filtered vectors fd, f, and w, and matrices E, Gd and G
        fd_filt = dx_filt
        f_filt = np.zeros((n_times, n_x))
        E = self.model.get_disturbance_prop_matrix()
        w_filt = np.zeros((n_times, n_w))
        Gd_filt = np.zeros((n_times, n_points_per_time, n_param))
        G_fun = ca.Function(
            "G_fun",
            [x_sym, u_sym, theta_sym],
            [self.model.state_update_ct_wm(x_sym, u_sym, theta_sym)],
        )
        G_fun_jac = ca.jacobian(G_fun(x_sym, u_sym, theta_sym), theta_sym)
        G_fun_jac_fun = ca.Function(
            "G_fun_jac_fun", [x_sym, u_sym, theta_sym], [G_fun_jac]
        )
        G_filt = np.array(
            [G_fun_jac_fun(x_filt[i, :], u_filt[i, :], 0) for i in range(n_times)]
        )

        # Compute filtered regression matrix A and vector b
        A = np.zeros((n_times * n_points_per_time, n_param))
        b = np.zeros((n_times * n_points_per_time, 1))
        for i in range(n_times):
            A[i * n_points_per_time : (i + 1) * n_points_per_time, :] = (
                Gd_filt[i, :, :] - G_filt[i, :, :]
            )
            b[i * n_points_per_time : (i + 1) * n_points_per_time, :] = (
                f_filt[i, x_wm_idc]
                + (E @ w_filt[i, :])[x_wm_idc]
                - fd_filt[i, x_wm_idc]
            ).reshape((-1, 1))

        # Estimate parameters using least squares
        theta_est = linalg.pinv(A) @ b

        # Return estimate of motor_tau
        return 1 / theta_est[0, 0]

    def estimate_motortau_gradient(self):
        G = self.inputs_int[:, 5:-5] - self.outputs_int[12:16, 5:-5]
        dwm = np.gradient(self.outputs_int[12:16, 5:-5], self.times_int[5:-5], axis=1)
        n_points = G.shape[0]
        n_times = G.shape[1]
        A = np.zeros((n_times * n_points, 1))
        b = np.zeros((n_times * n_points, 1))
        for i in range(n_times):
            A[i * n_points : (i + 1) * n_points] = G[:, i].reshape((-1, 1))
            b[i * n_points : (i + 1) * n_points] = dwm[:, i].reshape((-1, 1))
        theta_est = linalg.pinv(A) @ b
        return 1 / theta_est[0, 0]

    def estimate_inertia_matrix(self):
        # Set dimensions
        x_wb_idx_start = 9
        n_wb = 3
        n_param = 6

        # Get dimensions
        n_x = self.model.get_n_states()
        n_w = self.model.get_n_disturbances()
        n_points_per_time = n_wb
        n_times = self.outputs_int.shape[1] - self.filters_start_time

        # Set body angular velocity state indices
        x_wb_idc = np.linspace(x_wb_idx_start, x_wb_idx_start + n_wb - 1, n_wb).astype(
            int
        )

        # Compute states and inputs using the low-pass filter
        x_filt = np.array(
            [
                signal.lsim(self.lp_filter, U=row.T, T=self.times_int)[1]
                for row in self.outputs_int
            ]
        ).T
        x_filt = x_filt[self.filters_start_time :, :]
        u_filt = np.array(
            [
                signal.lsim(self.lp_filter, U=row.T, T=self.times_int)[1]
                for row in self.inputs_int
            ]
        ).T
        u_filt = u_filt[self.filters_start_time :, :]

        # Compute the derivative of the states using the high-pass filter
        dx_filt = np.array(
            [
                signal.lsim(self.hp_filter, U=row.T, T=self.times_int)[1]
                for row in self.outputs_int
            ]
        ).T
        dx_filt = dx_filt[self.filters_start_time :, :]

        # Create symbolic variables to define symbolic vectors and matrices
        x_sym = ca.MX.sym("x", n_x)
        u_sym = ca.MX.sym("u", self.model.get_n_inputs())
        dx_sym = ca.MX.sym("dx", n_x)
        theta_sym = ca.MX.sym("theta_sym", 6)

        # Compute filtered vectors fd, f, and w, and matrices E, Gd and G
        fd_filt = np.zeros((n_times, n_x))
        f_filt = np.zeros((n_times, n_x))
        for i in range(n_times):
            f_filt[i, :] = np.concatenate(
                (
                    np.zeros((x_wb_idx_start, 1)),
                    self.model.state_update_ct_compute_torque_t(
                        x_filt[i, :], u_filt[i, :]
                    ),
                )
            ).ravel()
            # f_filt[i, :] = np.concatenate(
            #     (
            #         np.zeros((x_wb_idx_start, 1)),
            #         self.model.state_update_ct_compute_torque_wm(
            #             x_filt[i, :], u_filt[i, :]
            #         ),
            #         np.zeros((n_x - x_wb_idx_start - n_wb, 1)),
            #     )
            # ).ravel()
        E = self.model.get_disturbance_prop_matrix()
        w_filt = np.zeros((n_times, n_w))
        Gd_fun = ca.Function(
            "Gd_fun",
            [dx_sym, theta_sym],
            [self.model.state_update_ct_wb_lh_side_vec(dx_sym, theta_sym)],
        )
        Gd_fun_jac = ca.jacobian(Gd_fun(x_sym, theta_sym), theta_sym)
        Gd_fun_jac_fun = ca.Function("Gd_fun_jac_fun", [x_sym, theta_sym], [Gd_fun_jac])
        Gd_filt = np.array(
            [
                Gd_fun_jac_fun(dx_filt[i, :], np.zeros((n_param, 1)))
                for i in range(n_times)
            ]
        )
        G_fun = ca.Function(
            "G_fun",
            [x_sym, u_sym, theta_sym],
            [self.model.state_update_ct_wb_rh_side_vec(x_sym, u_sym, theta_sym)],
        )
        G_fun_jac = ca.jacobian(G_fun(x_sym, u_sym, theta_sym), theta_sym)
        G_fun_jac_fun = ca.Function(
            "G_fun_jac_fun", [x_sym, u_sym, theta_sym], [G_fun_jac]
        )
        G_filt = np.array(
            [
                G_fun_jac_fun(x_filt[i, :], u_filt[i, :], np.zeros((n_param, 1)))
                for i in range(n_times)
            ]
        )

        # Compute filtered regression matrix A and vector b
        A = np.zeros((n_times * n_points_per_time, n_param))
        b = np.zeros((n_times * n_points_per_time, 1))
        for i in range(n_times):
            A[i * n_points_per_time : (i + 1) * n_points_per_time, :] = (
                Gd_filt[i, :, :] - G_filt[i, :, :]
            )
            b[i * n_points_per_time : (i + 1) * n_points_per_time, :] = (
                f_filt[i, x_wb_idc]
                + (E @ w_filt[i, :])[x_wb_idc]
                - fd_filt[i, x_wb_idc]
            ).reshape((-1, 1))

        # Estimate parameters using least squares
        theta_est = linalg.pinv(A) @ b

        # Return estimate of motor_tau
        return helpers.get_inertia_matrix_from_vec(theta_est)

    def estimate_drag_coefficients(self):
        # Set dimensions
        x_v_idx_start = 6
        n_v = 3
        n_param = 3

        # Get dimensions
        n_x = self.model.get_n_states()
        n_w = self.model.get_n_disturbances()
        n_points_per_time = n_v
        n_times = self.outputs_int.shape[1] - self.filters_start_time

        # Set motor velocity state indices
        x_v_idc = np.linspace(x_v_idx_start, x_v_idx_start + n_v - 1, n_v).astype(int)

        # Compute states and inputs using the low-pass filter
        x_filt = np.array(
            [
                signal.lsim(self.lp_filter, U=row.T, T=self.times_int)[1]
                for row in self.outputs_int
            ]
        ).T
        x_filt = x_filt[self.filters_start_time :, :]
        u_filt = np.array(
            [
                signal.lsim(self.lp_filter, U=row.T, T=self.times_int)[1]
                for row in self.inputs_int
            ]
        ).T
        u_filt = u_filt[self.filters_start_time :, :]

        # Compute the derivative of the states using the high-pass filter
        dx_filt = np.array(
            [
                signal.lsim(self.hp_filter, U=row.T, T=self.times_int)[1]
                for row in self.outputs_int
            ]
        ).T
        dx_filt = dx_filt[self.filters_start_time :, :]

        # Create symbolic variables to define symbolic vectors and matrices
        x_sym = ca.MX.sym("x", n_x)
        u_sym = ca.MX.sym("u", self.model.get_n_inputs())
        theta_sym = ca.MX.sym("theta_sym", 3)

        # Compute filtered vectors fd, f, and w, and matrices E, Gd and G
        fd_filt = dx_filt
        f_filt = np.zeros((n_times, n_x))
        for i in range(n_times):
            f_filt[i, :] = np.concatenate(
                (
                    np.zeros((x_v_idx_start, 1)),
                    self.model.state_update_ct_v_vec(
                        x_filt[i, :], u_filt[i, :], np.zeros((n_param, 1))
                    ),
                    np.zeros((n_x - x_v_idx_start - n_v, 1)),
                )
            ).ravel()
            # f_filt[i, :] = np.concatenate(
            #     (
            #         np.zeros((x_v_idx_start, 1)),
            #         self.model.state_update_ct_v_vec(
            #             x_filt[i, :], u_filt[i, :], np.zeros((n_param, 1))
            #         ),
            #         np.zeros((n_x - x_v_idx_start - n_v, 1)),
            #     )
            # ).ravel()
        E = self.model.get_disturbance_prop_matrix()
        w_filt = np.zeros((n_times, n_w))
        Gd_filt = np.zeros((n_times, n_points_per_time, n_param))
        G_fun = ca.Function(
            "G_fun",
            [x_sym, u_sym, theta_sym],
            [self.model.state_update_ct_v_vec(x_sym, u_sym, theta_sym)],
        )
        G_fun_jac = ca.jacobian(G_fun(x_sym, u_sym, theta_sym), theta_sym)
        G_fun_jac_fun = ca.Function(
            "G_fun_jac_fun", [x_sym, u_sym, theta_sym], [G_fun_jac]
        )
        G_filt = np.array(
            [
                G_fun_jac_fun(x_filt[i, :], u_filt[i, :], np.zeros((n_param, 1)))
                for i in range(n_times)
            ]
        )

        # Compute filtered regression matrix A and vector b
        A = np.zeros((n_times * n_points_per_time, n_param))
        b = np.zeros((n_times * n_points_per_time, 1))
        for i in range(n_times):
            A[i * n_points_per_time : (i + 1) * n_points_per_time, :] = (
                Gd_filt[i, :, :] - G_filt[i, :, :]
            )
            b[i * n_points_per_time : (i + 1) * n_points_per_time, :] = (
                f_filt[i, x_v_idc] + (E @ w_filt[i, :])[x_v_idc] - fd_filt[i, x_v_idc]
            ).reshape((-1, 1))

        # Estimate parameters using least squares
        theta_est = linalg.pinv(A) @ b

        # Return estimate of motor_tau
        return theta_est

    def process_recorded_data(self):
        # READ RECORDED DATA
        # -------------------------------------------------------------------------------
        # Store data in json file in dict
        with open(f"{self.json_dir}/{self.json_name}.json", "r") as openfile:
            json_data = json.load(openfile)
        self.inputs_times = np.array(json_data["/step_control"]["t"])
        wmc = np.array(json_data["/step_control"]["u"]).T
        n_inputs_times = len(self.inputs_times)
        self.inputs = np.zeros((4, n_inputs_times))
        for i in range(n_inputs_times):
            self.inputs[:, i] = self.model.motor_speeds_to_thrusts(wmc[:, i])

        self.outputs_times = np.array(json_data["/falcon/odometry"]["t"])
        self.outputs = np.array(json_data["/falcon/odometry"]["y"]).T
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
            # If not existing yet: select desired part of the data from plot
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
            print(f"Selected points: {sel}")
            print("You can close the data selection figure now.")
            print(f"sel: {sel}")

            select_data_dict[self.json_name] = sel
            with open(data_sel_path, "w") as outfile:
                json.dump(select_data_dict, outfile)
            print(f"Added sel for {self.json_name} to {self.data_sel_file_name}.")

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
        # -------------------------------------------------------------------------------

        # INTERPOLATE DATA
        # -------------------------------------------------------------------------------
        self.times_max_begin = max(self.outputs_times[0], self.inputs_times[0])
        self.times_min_end = min(self.outputs_times[-1], self.inputs_times[-1])
        self.times_int = np.arange(self.times_max_begin, self.times_min_end, self.ts)
        if self.exp_type == "sim" or self.exp_type == "gaz":
            self.times_int = np.round(
                self.times_int, 5
            )  # NOTE: same argument as before with inputs times

        f = interpolate.interp1d(self.inputs_times, self.inputs, kind="previous")
        self.inputs_int = f(self.times_int)
        f = interpolate.interp1d(self.outputs_times, self.outputs)
        self.outputs_int = f(self.times_int)

        self.times_int = self.times_int - self.times_int[0]
        self.n_times = len(self.times_int)
        # -------------------------------------------------------------------------------


if __name__ == "__main__":
    # Start timing
    start = time.time()

    # Log settings
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
    config_path = f"{config_dir}/scripts/determine_model_coefficients.yaml"
    data_dir = f"{package_dir}/data"
    data_sel_dir = f"{data_dir}/selected_data"
    data_sel_file_name = "model_id_data_select.json"

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    # Process json data files
    json_names = config["recorded_data"]["json_names"]
    for json_name in json_names:
        print()
        print("-" * 100)
        log.warning(f"Selected json file: {json_name}")

        # Process file name
        _, exp_type, _, _, exp_details = helpers.read_file_name(json_name[:-5])
        print(f"Experiment details: {exp_details}")

        determine_model_coefficients = DetermineModelCoefficients(
            config,
            config_dir,
            json_dir,
            json_name,
            data_sel_dir,
            data_sel_file_name,
            exp_type,
        )
        determine_model_coefficients.process_recorded_data()
        if "motortau" in exp_details:
            print(f"Identifying motor_tau coefficient...", end="\r")
            motor_tau_est = determine_model_coefficients.estimate_motortau()
            print(f"Identified motor_tau coefficient: {motor_tau_est}")
            motor_tau_est_gradient = (
                determine_model_coefficients.estimate_motortau_gradient()
            )
            print(
                f"Identified motor_tau coefficient using gradient: {motor_tau_est_gradient}"
            )
        if "inertia" in exp_details:
            print(f"Identifying inertia_matrix...", end="\r")
            inertia_matrix_est = determine_model_coefficients.estimate_inertia_matrix()
            print(f"Identified inertia_matrix: {inertia_matrix_est}")
        if "drag" in exp_details:
            print(f"Identifying drag_coefficients...", end="\r")
            drag_coefficients_est = (
                determine_model_coefficients.estimate_drag_coefficients()
            )
            print(f"Identified drag_coefficients: {drag_coefficients_est}")
        print("-" * 100)

    # End timing and print
    end = time.time()
    print(f"Elapsed time: {end - start}")
