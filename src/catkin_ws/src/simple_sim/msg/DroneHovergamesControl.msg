# Message with control input to simple simulation of hovergames drone model with and without yaw
# In case yaw is not used, the last state and last input are not used

Header header

# Inputs: phi_c, theta_c, dpsi_c, thrust_c
float64[4] u

# Note: thrust_c has unit [m/s^2] (real thrust divided by mass)
