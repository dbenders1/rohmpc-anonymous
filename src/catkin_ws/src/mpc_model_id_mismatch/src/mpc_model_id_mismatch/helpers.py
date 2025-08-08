import logging
import math
import os
import yaml

import numpy as np
import matplotlib.pyplot as plt

from acados_template import (
    AcadosModel,
    AcadosOcp,
    AcadosOcpConstraints,
    AcadosOcpCost,
    AcadosOcpDims,
    AcadosOcpOptions,
    AcadosOcpSolver,
)
from casadi import (
    cos,
    cross,
    diag,
    diagcat,
    horzcat,
    inv,
    mtimes,
    MX,
    reshape,
    sin,
    SX,
    tan,
    vertcat,
)
from scipy.spatial.transform import Rotation as R

FLOAT_TOL = 1e-6
INF = 1e8


def body_to_world(vec_body, eul):
    phi = eul[0]
    theta = eul[1]
    psi = eul[2]
    R_body_to_world = get_rot_matrix_coordinates(phi, theta, psi)
    return R_body_to_world @ vec_body


def get_acados_status_message(status):
    if status == 0:
        return "Success"
    elif status == 1:
        return "Failure"
    elif status == 2:
        return "Maximum iterations reached"
    elif status == 3:
        return "Minimum step size in QP solver reached"
    elif status == 4:
        return "QP solver failed"
    elif status == 5:
        return "Ready"
    else:
        return "Unknown status"


def get_acados_mhe_solver(
    model,
    M,
    ts,
    sim_eta_max,
    sim_eta_max_scaling=1,
    determine_w=False,
    generate_solver=True,
):
    # Obtain number of states and inputs
    nx = model.get_n_states()
    nu = model.get_n_inputs()
    ny = model.get_n_outputs()
    if determine_w:
        nw = model.get_n_states()
    else:
        nw = model.get_n_disturbances()
    neta = model.get_n_measurement_noises()
    F_transpose = model.get_measurement_noise_sel_matrix()
    # F_complement = model.get_wm_sel_matrix()

    # Define shooting variables
    x_est = SX.sym("x", nx)
    w_est = SX.sym("w", nw)

    # Define parameters
    u_applied = SX.sym("u_applied", nu)
    y_meas = SX.sym("y_meas", ny)

    # Create OCP dimensions
    ocp_dims = AcadosOcpDims()
    ocp_dims.N = M

    # Create acados model
    acados_model = AcadosModel()
    acados_model.name = model.get_name()
    acados_model.x = x_est
    acados_model.u = vertcat(w_est)
    acados_model.p = vertcat(u_applied, y_meas)
    # state update equality constraint
    acados_model.f_expl_expr = model.state_update_ct_noise(
        x_est, u_applied, w_est, determine_w
    )
    if determine_w:
        # symbolic cost terms
        acados_model.cost_y_expr_0 = w_est
        acados_model.cost_y_expr = w_est
        # symbolic constraint terms
        acados_model.con_h_expr_0 = y_meas - x_est
        acados_model.con_h_expr = y_meas - x_est
        acados_model.con_h_expr_e = y_meas - x_est
    else:
        # symbolic cost terms
        acados_model.cost_y_expr_0 = vertcat(
            w_est, mtimes(F_transpose, y_meas - model.get_outputs(x_est, u_applied))
        )
        acados_model.cost_y_expr = vertcat(
            w_est, mtimes(F_transpose, y_meas - model.get_outputs(x_est, u_applied))
        )
        acados_model.cost_y_expr_e = mtimes(
            F_transpose, y_meas - model.get_outputs(x_est, u_applied)
        )
        # symbolic constraint terms
        acados_model.con_h_expr_0 = mtimes(
            F_transpose, y_meas - model.get_outputs(x_est, u_applied)
        )
        acados_model.con_h_expr = mtimes(
            F_transpose, y_meas - model.get_outputs(x_est, u_applied)
        )
        acados_model.con_h_expr_e = mtimes(
            F_transpose, y_meas - model.get_outputs(x_est, u_applied)
        )
        # acados_model.con_h_expr_0 = mtimes(
        #     F_complement, y_meas - model.get_outputs(x_est, u_applied)
        # )
        # acados_model.con_h_expr = mtimes(
        #     F_complement, y_meas - model.get_outputs(x_est, u_applied)
        # )
        # acados_model.con_h_expr_e = mtimes(
        #     F_complement, y_meas - model.get_outputs(x_est, u_applied)
        # )

    # Create OCP cost
    ocp_cost = AcadosOcpCost()
    ocp_cost.cost_type_0 = "NONLINEAR_LS"
    ocp_cost.cost_type = "NONLINEAR_LS"
    if determine_w:
        ocp_cost.W_0 = np.zeros((nw, nw))
        ocp_cost.yref_0 = np.zeros((nw,))
        ocp_cost.W = np.zeros((nw, nw))
        ocp_cost.yref = np.zeros((nw,))
    else:
        ocp_cost.W_0 = np.zeros((nw + neta, nw + neta))
        ocp_cost.yref_0 = np.zeros((nw + neta,))
        ocp_cost.W = np.zeros((nw + neta, nw + neta))
        ocp_cost.yref = np.zeros((nw + neta,))
        ocp_cost.cost_type_e = "NONLINEAR_LS"
        ocp_cost.W_e = np.zeros((neta, neta))
        ocp_cost.yref_e = np.zeros((neta,))

    # Create OCP constraints
    # Known bounds on measurement noise
    ocp_constraints = AcadosOcpConstraints()
    if determine_w:
        # ocp_constraints.x0 = np.zeros((nx,))
        ocp_constraints.lh_0 = np.zeros((nw,))
        ocp_constraints.uh_0 = np.zeros((nw,))
        ocp_constraints.lh = np.zeros((nw,))
        ocp_constraints.uh = np.zeros((nw,))
        ocp_constraints.lh_e = np.zeros((nw,))
        ocp_constraints.uh_e = np.zeros((nw,))
    else:
        sim_eta_max = sim_eta_max_scaling * sim_eta_max
        ocp_constraints.lh_0 = -sim_eta_max
        ocp_constraints.uh_0 = sim_eta_max
        ocp_constraints.lh = -sim_eta_max
        ocp_constraints.uh = sim_eta_max
        ocp_constraints.lh_e = -sim_eta_max
        ocp_constraints.uh_e = sim_eta_max

    # Create OCP options object
    ocp_options = AcadosOcpOptions()
    ocp_options.tf = M * ts
    ocp_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp_options.hessian_approx = "GAUSS_NEWTON"
    ocp_options.integrator_type = "ERK"
    ocp_options.nlp_solver_type = "SQP"

    # Create OCP object
    # NOTE: no constraints are put on the shooting variables
    ocp = AcadosOcp()
    ocp.dims = ocp_dims
    ocp.model = acados_model
    ocp.cost = ocp_cost
    ocp.constraints = ocp_constraints
    ocp.solver_options = ocp_options
    ocp.parameter_values = np.zeros((acados_model.p.rows(),))

    # Create OCP solver
    solver_filename = f"acados_mhe_{M}.json"
    if not generate_solver and os.path.exists(solver_filename):
        ocp_solver = AcadosOcpSolver(
            ocp, json_file=solver_filename, build=False, generate=False
        )
        print(f"Loaded MHE solver from {solver_filename}")
    else:
        ocp_solver = AcadosOcpSolver(ocp, json_file=solver_filename)
        print(f"Generated MHE solver {solver_filename}")
    return ocp_solver


def get_cost_mhe(M, w_pred, eta_pred, Q_mhe, R_mhe, stage_idc):
    mhe_cost = 0
    n_w = w_pred.shape[0]
    for k in stage_idc:
        if k < M:
            mhe_cost += get_cost_stage(w_pred[:, k], eta_pred[:, k], Q_mhe, R_mhe)
        else:
            mhe_cost += get_cost_stage(np.zeros((n_w,)), eta_pred[:, k], Q_mhe, R_mhe)
    return mhe_cost


def get_cost_mle(M, w_pred, eta_pred, Q_mhe, R_mhe, Q, R, stage_idc):
    mle_cost = 0
    mle_cost += get_cost_mhe(M, w_pred, eta_pred, Q_mhe, R_mhe, stage_idc)
    logdet_Q_sign, logdet_Q_abs = np.linalg.slogdet(Q)
    logdet_Q = logdet_Q_sign * logdet_Q_abs
    logdet_R_sign, logdet_R_abs = np.linalg.slogdet(R)
    logdet_R = logdet_R_sign * logdet_R_abs
    for stage_idx in stage_idc:
        if stage_idx < M:
            mle_cost += logdet_Q
        mle_cost += logdet_R
    return mle_cost


def get_cost_stage(w, eta, Q_mhe, R_mhe):
    y_err = vertcat(w, eta)
    return 0.5 * mtimes(mtimes(y_err.T, diagcat(Q_mhe, R_mhe)), y_err)


def get_cost_terminal(eta, R_mhe):
    return 0.5 * mtimes(mtimes(eta.T, R_mhe), eta)


def get_diag_matrix_from_vec(diag_matrix_vec):
    return diag(diag_matrix_vec)


def get_inertia_matrix_from_vec(inertia_matrix_vec):
    inertia_matrix = vertcat(
        horzcat(inertia_matrix_vec[0], inertia_matrix_vec[1], inertia_matrix_vec[2]),
        horzcat(inertia_matrix_vec[1], inertia_matrix_vec[3], inertia_matrix_vec[4]),
        horzcat(inertia_matrix_vec[2], inertia_matrix_vec[4], inertia_matrix_vec[5]),
    )
    return inertia_matrix


def get_matrices_with_input_delay(A, B, C, D, nk_input_delay):
    nx = A.shape[0]

    if nk_input_delay == 1:
        A_del0 = np.concatenate((A, B), axis=1)
        A_del1 = np.zeros((1, nx + 1))
    else:
        A_del0 = np.concatenate((A, B, np.zeros((nx, nk_input_delay - 1))), axis=1)
        A_del1 = np.concatenate(
            (np.zeros((nk_input_delay, nx)), np.diag(np.ones(nk_input_delay - 1), k=1)),
            axis=1,
        )

    A_del = np.concatenate((A_del0, A_del1), axis=0)
    B_del = np.concatenate(
        (np.zeros((nx + nk_input_delay - 1, 1)), np.array([[1]])), axis=0
    )
    C_del = np.concatenate((C, np.zeros((1, nk_input_delay))), axis=1)
    D_del = D

    return A_del, B_del, C_del, D_del


def get_mhe_weighting_matrices(Q, R, eps):
    nw = Q.shape[0]
    neta = R.shape[0]
    # First subtract eps from the covariance matrices to ensure correct inversion in case the diagonal elements are larger than self.eps
    # Q = subtract_eps_from_diag_and_clip_at_zero(Q, eps)
    # R = subtract_eps_from_diag_and_clip_at_zero(R, eps)
    Q_mhe = np.linalg.inv(Q + eps * np.eye(nw))
    R_mhe = np.linalg.inv(R + eps * np.eye(neta))
    return Q_mhe, R_mhe


def get_rot_matrix_coordinates(phi, theta, psi):
    R_psi = vertcat(
        horzcat(cos(psi), -sin(psi), 0),
        horzcat(sin(psi), cos(psi), 0),
        horzcat(0, 0, 1),
    )
    R_theta = vertcat(
        horzcat(cos(theta), 0, sin(theta)),
        horzcat(0, 1, 0),
        horzcat(-sin(theta), 0, cos(theta)),
    )
    R_phi = vertcat(
        horzcat(1, 0, 0),
        horzcat(0, cos(phi), -sin(phi)),
        horzcat(0, sin(phi), cos(phi)),
    )
    return mtimes(mtimes(R_psi, R_theta), R_phi)


def get_rot_matrix_rates(phi, theta):
    return vertcat(
        horzcat(1, sin(phi) * tan(theta), cos(phi) * tan(theta)),
        horzcat(0, cos(phi), -sin(phi)),
        horzcat(0, sin(phi) / cos(theta), cos(phi) / cos(theta)),
    )


def get_tube_size_over_time(n_pred_steps, dt, w_bar_c, rho_c):
    s = np.zeros(n_pred_steps)
    for k_idx in range(n_pred_steps):
        s[k_idx] = (1 - math.exp(-rho_c * k_idx * dt)) * w_bar_c / rho_c
    return s


def quat_mult(q1, q2):
    ans = vertcat(
        q2[0, :] * q1[0, :]
        - q2[1, :] * q1[1, :]
        - q2[2, :] * q1[2, :]
        - q2[3, :] * q1[3, :],
        q2[0, :] * q1[1, :]
        + q2[1, :] * q1[0, :]
        - q2[2, :] * q1[3, :]
        + q2[3, :] * q1[2, :],
        q2[0, :] * q1[2, :]
        + q2[2, :] * q1[0, :]
        + q2[1, :] * q1[3, :]
        - q2[3, :] * q1[1, :],
        q2[0, :] * q1[3, :]
        - q2[1, :] * q1[2, :]
        + q2[2, :] * q1[1, :]
        + q2[3, :] * q1[0, :],
    )
    return ans


def read_file_name(file_name):
    items = file_name.split("_")

    exp_date = items[0]
    exp_type = items[1]
    exp_platform = items[2]
    exp_goal = "_".join(items[3:6])
    exp_details = items[6:]

    return exp_date, exp_type, exp_platform, exp_goal, exp_details


def resize_fig(fig, scale=1):
    width_in_inches = 245.71811 / 72
    orig_size = fig.get_size_inches()
    aspect_ratio = scale * orig_size[1] / orig_size[0]
    fig.set_size_inches(width_in_inches, width_in_inches * aspect_ratio)
    new_size = fig.get_size_inches()


def rotate_quat(q1, v1):
    ans = quat_mult(
        quat_mult(q1, vertcat(0, v1)),
        vertcat(q1[0, :], -q1[1, :], -q1[2, :], -q1[3, :]),
    )
    return vertcat(ans[1, :], ans[2, :], ans[3, :])  # to covert to 3x1 vec


def rotate_quat_inverse(q1, v1):
    ans = quat_mult(
        quat_mult(vertcat(q1[0, :], -q1[1, :], -q1[2, :], -q1[3, :]), vertcat(0, v1)),
        q1,
    )
    return vertcat(ans[1, :], ans[2, :], ans[3, :])  # to covert to 3x1 vec


def rot_matrix_zyx(rpy_angles):
    phi = rpy_angles[0]
    theta = rpy_angles[1]
    psi = rpy_angles[2]
    return np.array(
        [
            [
                math.cos(theta) * math.cos(psi),
                math.sin(phi) * math.sin(theta) * math.cos(psi)
                - math.cos(phi) * math.sin(psi),
                math.cos(phi) * math.sin(theta) * math.cos(psi)
                + math.sin(phi) * math.sin(psi),
            ],
            [
                math.cos(theta) * math.sin(psi),
                math.sin(phi) * math.sin(theta) * math.sin(psi)
                + math.cos(phi) * math.cos(psi),
                math.cos(phi) * math.sin(theta) * math.sin(psi)
                - math.sin(phi) * math.cos(psi),
            ],
            [
                -math.sin(theta),
                math.sin(phi) * math.cos(theta),
                math.cos(phi) * math.cos(theta),
            ],
        ]
    )


def save_fig(fig, fig_path, transparent=False):
    file_type = fig_path[-3:]
    fig.savefig(fig_path, format=f"{file_type}", transparent=transparent)


def set_fig_properties():
    props = dict()
    props["titlepad"] = 4
    props["tickpad"] = 1
    props["xlabelpad"] = 0
    props["ylabelpad"] = 1
    props["zlabelpad"] = 0
    props["textsize"] = plt.rcParams["xtick.labelsize"]
    return props


def set_plt_properties():
    # Plot settings
    fontsize = 10
    plt.rcParams["font.family"] = "serif"
    plt.rcParams["font.serif"] = ["Times New Roman"]
    plt.rcParams["text.usetex"] = True
    plt.rcParams["text.latex.preamble"] = r"\usepackage{bm}"
    plt.rcParams["font.size"] = fontsize
    plt.rcParams["axes.labelsize"] = fontsize - 2
    plt.rcParams["axes.titlesize"] = fontsize
    plt.rcParams["xtick.labelsize"] = fontsize - 4
    plt.rcParams["ytick.labelsize"] = fontsize - 4
    plt.rcParams["legend.fontsize"] = fontsize - 4
    # Set dpi for saving figures
    plt.rcParams["savefig.dpi"] = 300
    plt.rcParams["figure.dpi"] = 300


def set_start_time_to_zero(*args):
    result = []
    start_time = np.inf

    for array in args:
        start_time = min(start_time, array[0])

    for array in args:
        array = array - start_time
        result.append(array)

    return result


def solve_rk4(state_update_ct, state_cur, u, dt):
    k1 = state_update_ct(state_cur, u)
    k2 = state_update_ct(state_cur + dt / 2 * k1, u)
    k3 = state_update_ct(state_cur + dt / 2 * k2, u)
    k4 = state_update_ct(state_cur + dt * k3, u)
    return state_cur + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def solve_rk4_noise(state_update_ct_noise, state_cur, u, w, dt):
    k1 = state_update_ct_noise(state_cur, u, w)
    k2 = state_update_ct_noise(state_cur + dt / 2 * k1, u, w)
    k3 = state_update_ct_noise(state_cur + dt / 2 * k2, u, w)
    k4 = state_update_ct_noise(state_cur + dt * k3, u, w)
    return state_cur + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def subtract_eps_from_diag_and_clip_at_zero(matrix, eps):
    """
    Subtracts eps from the diagonal elements of the matrix and clips the result at zero.
    """
    # Create a copy of the matrix to avoid modifying the original
    matrix_copy = np.copy(matrix)

    # Subtract eps from the diagonal elements
    np.fill_diagonal(matrix_copy, np.clip(np.diagonal(matrix_copy) - eps, 0, None))

    return matrix_copy


class DroneAgiModel:
    def __init__(self, quad_name, g, params_file):
        self.logger = logging.getLogger(__name__)
        self.logger.info(f"{self.__class__.__name__}.__init__")

        # Set name
        self.quad_name = quad_name

        # Set gravity
        self.g = g

        # Set parameters from file
        with open(params_file, "r") as file:
            params = yaml.safe_load(file)
        for key, value in params.items():
            setattr(self, key, value)

        # Process parameters
        self.inertia_matrix = np.array(self.inertia)
        self.inertia_matrix_inv = np.linalg.inv(self.inertia_matrix)
        self.motor_tau_inv = 1 / self.motor_tau
        self.t_bm = np.array([self.tbm_fr, self.tbm_bl, self.tbm_br, self.tbm_fl]).T
        self.B_allocation = np.array(
            [
                [1, 1, 1, 1],
                self.t_bm[1, :],
                -self.t_bm[0, :],
                self.kappa * np.array([-1, -1, 1, 1]),
            ]
        )
        self.kd = np.array(self.kd)

        # Set model, input, state, output and disturbance properties
        self.name = f"{self.quad_name}_t"
        self.input_names = ["t0c", "t1c", "t2c", "t3c"]
        self.nu = len(self.input_names)
        self.state_names = [
            "px",
            "py",
            "pz",
            "phi",
            "theta",
            "psi",
            "vx",
            "vy",
            "vz",
            "wbx",
            "wby",
            "wbz",
        ]
        self.nx = len(self.state_names)
        self.nw = self.nx
        self.E = np.concatenate(
            (np.zeros((self.nx - self.nw, self.nw)), np.eye(self.nw))
        )
        self.disturbance_idc = np.where(self.E.any(axis=1))[0]
        self.disturbance_names = [self.state_names[i] for i in self.disturbance_idc]
        # Get outputs by selecting from state vector the indices that correspond to column indices in C that contain a 1
        self.ny = self.nx
        self.C = np.eye(self.ny)
        self.output_idc = np.where(self.C.any(axis=0))[0]
        self.output_names = [self.state_names[i] for i in self.output_idc]
        # Get measurement noises by selecting from output vector the indices that correspond to row indices in F that contain a 1
        self.neta = self.ny
        self.F = np.eye(self.neta)
        self.measurement_noise_idc = np.where(self.F.any(axis=1))[0]
        self.measurment_noise_names = [
            self.output_names[i] for i in self.measurement_noise_idc
        ]
        self.F_complement = np.concatenate(
            (np.zeros((self.ny - self.neta, self.neta)), np.eye(self.ny - self.neta)),
            axis=1,
        )

        # Set system constraint bounds
        self.p_max = 4
        self.p_min = -self.p_max
        self.att_max = 0.1  # tuned value for feasible sdp for traj_circle_r1_f0dot3_cw
        self.att_min = -self.att_max
        self.v_max = 2
        self.v_min = -self.v_max
        self.wb_max = 0.3  # tuned value for feasible sdp for traj_circle_r1_f0dot3_cw
        self.wb_min = -self.wb_max
        self.t_var = 0.12
        self.t_hover = self.mass * self.g / 4
        self.t_min = np.max(
            [self.t_hover - self.t_var, self.thrust_map[0] * self.motor_omega_min**2]
        )
        self.t_max = np.min(
            [self.t_hover + self.t_var, self.thrust_map[0] * self.motor_omega_max**2]
        )

    def get_B_allocation(self):
        return self.B_allocation

    def get_hidden_state_idc(self):
        return [
            idx
            for idx, name in enumerate(self.state_names)
            if name not in self.output_names
        ]

    def get_input_names(self):
        return self.input_names

    def get_input_lower_bounds(self):
        return np.array([0, 0, 0, 0]).T

    def get_input_upper_bounds(self):
        return np.array([8.5, 8.5, 8.5, 8.5]).T

    def get_disturbance_idc(self):
        return self.disturbance_idc

    def get_disturbance_names(self):
        return self.disturbance_names

    def get_disturbance_prop_matrix(self):
        return self.E

    def get_disturbance_sel_matrix(self):
        return self.E.T

    def get_inertia_matrix(self):
        return self.inertia_matrix

    def get_measurement_matrix(self):
        return self.C

    def get_measurement_noise_names(self):
        return self.measurement_noise_names

    def get_measurement_noise_prop_matrix(self):
        return self.F

    def get_measurement_noise_sel_matrix(self):
        return self.F.T

    def get_model_data(self):
        return {
            "name": self.name,
            "nx": self.nx,
            "nu": self.nu,
            "ny": self.ny,
            "nw": self.nw,
            "neta": self.neta,
            "mass": self.mass,
            "inertia": self.inertia_matrix.tolist(),
            "B_allocation": self.B_allocation.tolist(),
            "motor_tau": self.motor_tau,
            "thrust_map": self.thrust_map,
            "kd": self.kd.tolist(),
            "p_min": self.p_min,
            "p_max": self.p_max,
            "att_min": self.att_min,
            "att_max": self.att_max,
            "v_min": self.v_min,
            "v_max": self.v_max,
            "wb_min": self.wb_min,
            "wb_max": self.wb_max,
            "t_hover": self.t_hover,
            "t_min": self.t_min,
            "t_max": self.t_max,
            "E": self.E.tolist(),
            "C": self.C.tolist(),
            "F": self.F.tolist(),
        }

    def get_wm_sel_matrix(self):
        return self.F_complement

    def get_name(self):
        return self.name

    def get_n_hidden_states(self):
        return self.nx - self.ny

    def get_n_inputs(self):
        return self.nu

    def get_n_disturbances(self):
        return self.nw

    def get_n_measurement_noises(self):
        return self.neta

    def get_n_outputs(self):
        return self.ny

    def get_n_states(self):
        return self.nx

    def get_output_idc(self):
        return self.output_idc

    def get_output_names(self):
        return self.output_names

    def get_outputs(self, x, u):
        return self.C @ x

    def get_outputs_noise(self, x, u, eta):
        return self.get_outputs(x, u) + self.F @ eta

    def get_quad_name(self):
        self.quad_name

    def get_state_names(self):
        return self.state_names

    def motor_speeds_to_thrusts(self, motor_speeds):
        return (
            self.thrust_map[0] * np.multiply(motor_speeds, motor_speeds)
            + self.thrust_map[1] * motor_speeds
            + self.thrust_map[2]
        )

    def state_update_ct(self, x, u):
        # Extract states from states vector
        x = vertcat(x)
        phi = x[3]
        theta = x[4]
        v = x[6:9]
        wb = x[9:12]

        # Compute rotation matrix to convert body rates to Euler angle rates
        Rr = get_rot_matrix_rates(phi, theta)

        # Return state derivatives
        return vertcat(
            v,
            mtimes(Rr, wb),
            self.state_update_ct_v(x, u, self.kd),
            mtimes(
                self.inertia_matrix_inv,
                self.state_update_ct_wb_rh_side(x, u, self.inertia_matrix),
            ),
        )

    def state_update_ct_compute_thrust_t(self, x, u):
        return mtimes(reshape(self.B_allocation[0, :], (1, 4)), u)

    def state_update_ct_compute_thrust_wm(self, x, u):
        return mtimes(
            reshape(self.B_allocation[0, :], (1, 4)),
            self.motor_speeds_to_thrusts(x[12:16]),
        )

    def state_update_ct_compute_torque_t(self, x, u):
        return mtimes(self.B_allocation[1:, :], u)

    def state_update_ct_compute_torque_wm(self, x, u):
        return mtimes(self.B_allocation[1:, :], self.motor_speeds_to_thrusts(x[12:16]))

    def state_update_ct_noise(self, x, u, w, all_states_disturbed=False):
        if all_states_disturbed:
            return self.state_update_ct(x, u) + np.eye(self.nx) @ w
        else:
            return self.state_update_ct(x, u) + self.E @ w

    def state_update_ct_v(self, x, u, kd):
        # Extract states from states vector
        x = vertcat(x)
        phi = x[3]
        theta = x[4]
        psi = x[5]
        v = x[6:9]

        # Compute thrust
        t = self.state_update_ct_compute_thrust_t(x, u)

        # Compute rotation matrix to convert coordinates from body to inertial frame
        Rc = get_rot_matrix_coordinates(phi, theta, psi)

        return vertcat(0, 0, -self.g) + mtimes(
            Rc, vertcat(0, 0, t / self.mass) - mtimes(kd, mtimes(Rc.T, v))
        )

    def state_update_ct_v_vec(self, x, u, kd_vec):
        return self.state_update_ct_v(x, u, get_diag_matrix_from_vec(kd_vec))

    def state_update_ct_wb_lh_side_vec(self, dx, inertia_matrix_vec):
        dwb = dx[9:12]
        return mtimes(get_inertia_matrix_from_vec(inertia_matrix_vec), dwb)

    def state_update_ct_wb_rh_side(self, x, u, inertia_matrix):
        # Extract states from states vector
        x = vertcat(x)
        wb = x[9:12]

        # Compute torque
        tau = self.state_update_ct_compute_torque_t(x, u)

        return tau - cross(wb, mtimes(inertia_matrix, wb))

    def state_update_ct_wb_rh_side_vec(self, x, u, inertia_matrix_vec):
        return self.state_update_ct_wb_rh_side(
            x, u, get_inertia_matrix_from_vec(inertia_matrix_vec)
        )

    def state_update_ct_wm(self, x, u, motor_tau_inv):
        # Extract states from states vector
        x = vertcat(x)
        wm = x[12:16]

        return (u - wm) * motor_tau_inv

    def thrusts_to_motor_speeds(self, thrusts):
        scale = 1 / (2 * self.thrust_map[0])
        offset = -self.thrust_map[1] * scale
        root = np.sqrt(
            self.thrust_map[1] ** 2
            - 4 * self.thrust_map[0] * (self.thrust_map[2] - thrusts)
        )
        return offset + scale * root
