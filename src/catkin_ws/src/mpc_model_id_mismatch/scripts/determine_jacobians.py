import casadi as ca
import numpy as np
import yaml
from os import path
from pathlib import Path

from mpc_model_id_mismatch import helpers

if __name__ == "__main__":
    # User settings
    package_dir = Path(__file__).parents[1]
    config_dir = f"{package_dir}/config"
    config_path = f"{config_dir}/scripts/determine_jacobians.yaml"

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    # Create model
    quad_name = config["model"]["name"]
    g = config["constants"]["g"]
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

    # Define symbolic variables
    nx = 12  # number of states
    nu = 4  # number of inputs
    x = ca.SX.sym("x", nx)
    u = ca.SX.sym("u", nu)

    # Define your dynamics symbolically
    f = model.state_update_ct(x, u)  # This must return a CasADi MX or SX expression

    # Compute Jacobians
    A = ca.jacobian(f, x)
    B = ca.jacobian(f, u)

    # Print the Jacobians
    print(f"A:\n{A}")
    print(f"B:\n{B}")
