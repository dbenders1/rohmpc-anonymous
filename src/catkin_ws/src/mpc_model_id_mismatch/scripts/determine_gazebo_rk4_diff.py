import json
import numpy as np
import sys
import yaml

from mpc_model_id_mismatch import helpers
from pathlib import Path
from os import path

if __name__ == "__main__":
    # Print settings
    np.set_printoptions(linewidth=np.inf, precision=10)

    # Load tmp_tio_data.json
    package_dir = Path(__file__).parents[1]
    config_dir = f"{package_dir}/config"
    config_path = f"{config_dir}/scripts/determine_gazebo_rk4_diff.yaml"
    data_dir = f"{package_dir}/data"
    model_mismatch_results_dir = f"{data_dir}/model_mismatch_results"

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    # Get common parameters
    quad_name = config["model"]["name"]
    g = config["constants"]["g"]
    gazebo_w_json_name = config["gazebo_w_json_name"]

    # Create model
    params_file = f"{config_dir}/systems/{quad_name}.yaml"
    if path.exists(params_file):
        print(f"Selected {quad_name} params file")
    else:
        print(f"Unknown quad name {quad_name}! Exiting")
        exit(1)
    if quad_name == "falcon":
        model = helpers.DroneAgiModel(quad_name, g, params_file)
    else:
        print(f"Unknown model {quad_name}! Exiting")
        exit(1)

    # Load data
    w_json_path = f"{model_mismatch_results_dir}/{gazebo_w_json_name}.json"
    if not Path(w_json_path).exists():
        print(
            f"File {w_json_path} does not exist. Please run the script that generates this file first."
        )
        sys.exit(1)
    with open(w_json_path, "r") as file:
        data = json.load(file)

    # Extract time, inputs, and outputs
    # Note: assuming that the outputs equal the states
    t = np.array(data["t"])
    u = np.array(data["u"])
    x = np.array(data["y"])
    w = np.array(data["w"])

    # Determine point where selected w entry is maximum
    w_idx = 3
    w_idx_max = np.max(w[w_idx, :])
    w_idx_max_idx = np.argmax(w[w_idx, :])
    print(f"w_idx_max: {w_idx_max}")
    print(f"w_idx_max_idx: {w_idx_max_idx}")

    # Set input to the exact input used in Gazebo
    u[:, w_idx_max_idx] = np.array([1.51810041, 1.503619112, 1.511563512, 1.522088392])

    # Forward-simulate the system using RK4
    dt = t[1] - t[0]
    n_times = len(t)
    n_u = u.shape[0]
    n_x = x.shape[0]
    x_rk4 = np.zeros((n_x, n_times))
    x_rk4[:, 0] = x[:, 0]  # Initial condition
    for i in range(1, n_times):
        x_rk4[:, i] = np.array(
            helpers.solve_rk4(model.state_update_ct, x[:, i - 1], u[:, i - 1], dt)
        ).flatten()

    # Print values of u and where w_wbx is maximum
    idx_before = w_idx_max_idx
    idx_after = idx_before + 1
    print(f"\nt before: {t[idx_before]}")
    print(f"x before: {x[:, idx_before]}")
    print(f"\nx_rk4 before: {x_rk4[:, idx_before]}")
    print(f"u during: {u[:, idx_before]}")
    drag_torque = model.B_allocation[3, :] * u[:, idx_before]
    print(f"drag_torque: {drag_torque}")
    print(f"\nt after: {t[idx_after]}")
    print(f"x after: {x[:, idx_after]}")
    print(f"\nx_rk4 after: {x_rk4[:, idx_after]}")
