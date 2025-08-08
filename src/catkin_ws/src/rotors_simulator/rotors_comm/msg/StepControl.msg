# Message with number of steps to run the simulation and system inputs to use during these steps

Header header

# Number of steps to run the simulation, according to https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1280
uint32 steps

# Inputs: wm0c,wm1c,wm2c,wm3c
float64[4] angular_velocities
