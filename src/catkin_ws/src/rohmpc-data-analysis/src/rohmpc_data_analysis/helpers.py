import matplotlib.pyplot as plt
import numpy as np

from enum import Enum
from matplotlib.legend_handler import HandlerPatch
from matplotlib.patches import Circle

FLOAT_TOL = 1e-6
NU = 4
NX = 12


class Colors(Enum):
    GREY = (0.859375, 0.859375, 0.859375)
    ORANGE = (1, 0.6484375, 0)
    PURPLE = (0.5, 0, 0.5)
    VIRIDIS_0 = (0.21875, 0.347656, 0.546875)


class HandlerCircle(HandlerPatch):
    def create_artists(
        self, legend, orig_handle, xdescent, ydescent, width, height, fontsize, trans
    ):
        center = 0.5 * width - 0.5 * xdescent, 0.5 * height - 0.5 * ydescent
        p = Circle(xy=center, radius=height / 2)
        self.update_prop(p, orig_handle, legend)
        p.set_transform(trans)
        return [p]


def compute_alpha(alpha_min, alpha_max, t, t_end):
    return alpha_max - (alpha_max - alpha_min) * (t / t_end) if t_end > 0 else alpha_max


def compute_rectangle_vertices(pos, yaw_angle, length, width):
    # Compute half-length and half-width
    half_length = length / 2
    half_width = width / 2

    # Define local coordinates of rectangle vertices in clockwise direction
    local_vertices = np.array(
        [
            [half_width, -half_width, -half_width, half_width],
            [half_length, half_length, -half_length, -half_length],
        ]
    )

    # Apply rotation and translation to get global coordinates
    global_vertices = compute_rotation_matrix_2d(yaw_angle) @ local_vertices + np.array(
        [[pos[0]], [pos[1]]]
    )

    return global_vertices


# Compute obstacle vertices, including robot region
# For rectangles, this means computing vertices of 2 more rectangles:
# - Rectangle with length = length + 2 * robot radius
# - Rectangle with width = width + 2 * robot radius
def compute_inflated_obstacle_vertices(obs_list, robot_radius):
    obs_verts = []
    for obs in obs_list:
        verts = dict()

        obs_pos = obs["p"]
        obs_att = obs["eul"]
        obs_dim = obs["dim"]

        verts["orig"] = compute_rectangle_vertices(
            obs_pos, obs_att[2], length=obs_dim[0], width=obs_dim[1]
        ).T
        verts["long"] = compute_rectangle_vertices(
            obs_pos, obs_att[2], length=obs_dim[0] + 2 * robot_radius, width=obs_dim[1]
        ).T
        verts["wide"] = compute_rectangle_vertices(
            obs_pos, obs_att[2], length=obs_dim[0], width=obs_dim[1] + 2 * robot_radius
        ).T

        obs_verts.append(verts)
    return obs_verts


def compute_pobj(u_pred_traj, x_pred_traj, u_ref_traj, x_ref_traj, N, Q_dt, R_dt, P_dt):
    pobj = 0
    for k in range(N):
        pobj += (
            (x_pred_traj[k, :] - x_ref_traj[k, :])
            @ Q_dt
            @ (x_pred_traj[k, :] - x_ref_traj[k, :])
        )
        pobj += (
            (u_pred_traj[k, :] - u_ref_traj[k, :])
            @ R_dt
            @ (u_pred_traj[k, :] - u_ref_traj[k, :])
        )
    pobj += (
        (x_pred_traj[N, :] - x_ref_traj[N, :])
        @ P_dt
        @ (x_pred_traj[N, :] - x_ref_traj[N, :])
    )
    return pobj


def compute_rotation_matrix_2d(angle):
    return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])


def compute_track_err(
    u_pred_traj, x_pred_traj, u_ref_traj, x_ref_traj, N, u_idc, x_idc
):
    Q_dt = np.zeros((NX, NX))
    Q_dt[x_idc, x_idc] = 1
    R_dt = np.zeros((NU, NU))
    R_dt[u_idc, u_idc] = 1
    P_dt = np.zeros((NX, NX))
    P_dt[x_idc, x_idc] = 1
    track_err = compute_pobj(
        u_pred_traj, x_pred_traj, u_ref_traj, x_ref_traj, N, Q_dt, R_dt, P_dt
    )
    return track_err


def compute_tube(trajectory, r):
    n_trajectory = len(trajectory)

    if np.isscalar(r):
        r = np.full(n_trajectory, r)

    if len(r) != n_trajectory:
        raise ValueError(
            "Tube radius must be a scalar or an array with the same length as the trajectory."
        )

    # Compute averaged direction vectors and normals
    normals = []
    for i in range(n_trajectory):
        if i == 0:
            direction = trajectory[i + 1] - trajectory[i]
        elif i == n_trajectory - 1:
            direction = trajectory[i] - trajectory[i - 1]
        else:
            direction = (trajectory[i + 1] - trajectory[i - 1]) / 2
        # Compute normal vector
        normal = np.array([-direction[1], direction[0]])
        normal = normal / np.linalg.norm(normal)
        normals.append(normal)
    normals = np.array(normals)

    # Compute outer and inner boundaries
    inner_boundary = []
    outer_boundary = []
    for i in range(n_trajectory):
        inner_boundary.append(trajectory[i] - r[i] * normals[i])
        outer_boundary.append(trajectory[i] + r[i] * normals[i])
    inner_boundary = np.array(inner_boundary)
    outer_boundary = np.array(outer_boundary)
    tube = np.stack([inner_boundary, outer_boundary], axis=0)

    return tube


def resize_fig(fig, scale=1):
    width_in_inches = 245.71811 / 72
    orig_size = fig.get_size_inches()
    aspect_ratio = scale * orig_size[1] / orig_size[0]
    fig.set_size_inches(width_in_inches, width_in_inches * aspect_ratio)
    new_size = fig.get_size_inches()


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
