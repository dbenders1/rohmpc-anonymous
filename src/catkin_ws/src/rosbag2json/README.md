# rosbag2json

ROS-Python package to save custom rosbag data in json format to be used in other repositories.

## Install

To run [rosbag2json.py](scripts/rosbag2json.py), we use the following packages globally on the system:

- [ROS Noetic](https://wiki.ros.org/noetic) (custom message [DroneFalconOutput](../simple_sim/msg/DroneFalconOutput.msg) needs to be known by building the [simple_sim](../simple_sim) package)
- [Python 3.8.10](https://www.python.org/downloads/release/python-3810/)

## Run

Adjust the settings in [rosbag2json.yaml](config/scripts/rosbag2json.yaml) and convert the rosbag file to a json file using the script [rosbag2json.py](scripts/rosbag2json.py). We recommend to run this script using VS Code by selecting the correct Python interpreter and pressing CTRL+F5.

This script will create json files in the [data/converted_bags](data/converted_bags) folder. If these files already exist, you do not have to run this conversion script. Otherwise, you will overwrite the existing files.
