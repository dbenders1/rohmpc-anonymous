# Message with number of steps to run the simulation and system inputs of simple simulation of Falcon drone model to use during these steps

Header header

# Number of steps to run the simulation, should be >= 1
uint32 steps

# Inputs: wm0c,wm1c,wm2c,wm3c
float64[4] angular_velocities
