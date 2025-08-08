import argparse
import json
import logging
import yaml

import numpy as np

from os import path
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
    config_dir = f"{package_dir}/config"
    config_path = f"{config_dir}/scripts/check_w_mhe_all.yaml"
    data_dir = f"{package_dir}/data"
    json_dir = f"{data_dir}/model_mismatch_results"
    if not path.exists(json_dir):
        log.warning(
            f"Directory {json_dir} does not exist! Please ensure that the model mismatch results are available"
        )
        exit(1)

    # Read configuration parameters
    with open(config_path) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
    w_json_name = config["w_json_name"]
    atol = config["atol"]
    rtol = config["rtol"]

    # Read json data
    w_json_path = f"{json_dir}/{w_json_name}.json"
    with open(w_json_path, "r") as f:
        data = json.load(f)
    w_mhe_all = np.array(data["w_mhe_all"])
    time_idc = []
    stage_idc = []
    for time_idx in range(w_mhe_all.shape[0] - 1):
        for stage_idx in range(w_mhe_all.shape[2] - 1):
            if not np.allclose(
                w_mhe_all[time_idx, :, stage_idx + 1],
                w_mhe_all[time_idx + 1, :, stage_idx],
                atol=atol,
                rtol=rtol,
            ):
                print(
                    f"w_mhe_all[{time_idx}, :, {stage_idx + 1}] is not approximately equal to w_mhe_all[{time_idx + 1}, :, {stage_idx}]"
                )
                time_idc.append(time_idx)
                stage_idc.append(stage_idx)

    print()
    for time_idx, stage_idx in zip(time_idc, stage_idc):
        print(
            f"w_mhe_all[{time_idx}, :, {stage_idx + 1}]: {w_mhe_all[time_idx, :, stage_idx + 1]}"
        )
        print(
            f"w_mhe_all[{time_idx + 1}, :, {stage_idx}]: {w_mhe_all[time_idx + 1, :, stage_idx]}"
        )
        print(
            f"np.allclose(w_mhe_all[{time_idx}, :, {stage_idx + 1}], w_mhe_all[{time_idx + 1}, :, {stage_idx}], atol={atol}, rtol={rtol}): {np.allclose(w_mhe_all[time_idx, :, stage_idx + 1], w_mhe_all[time_idx + 1, :, stage_idx], atol=atol, rtol=rtol)}"
        )
        print()
