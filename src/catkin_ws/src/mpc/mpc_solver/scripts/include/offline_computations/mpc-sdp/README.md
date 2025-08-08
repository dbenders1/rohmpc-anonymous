# mpc-sdp

This repository implements the offline computation of the robust reference tracking MPC formulation: computing robust and terminal ingredients using an SDP. The formulation is implemented in a modular way, so it can easily be adjusted for other types of nonlinear system models. Refer to [How to customize?](#how-to-customize) for more information.

Contents:\
[Install](#install)\
[Run](#run)\
[How to customize?](#how-to-customize)

## Install

To run the code in this repository, we use [MATLAB R2023a](https://nl.mathworks.com/products/new_products/release2023a.html) with the following packages:

- [CasADi](https://web.casadi.org/get)
- [YALMIP](https://yalmip.github.io) via [tbxmanager](https://tbxmanager.com)
- [SDPT3](https://blog.nus.edu.sg/mattohkc/softwares/sdpt3)
- [Mosek](https://www.mosek.com/documentation)

> :information_source: The robust and terminal ingredients are already computed and saved as [offline_design.mat](./offline_design.mat) in this repository. These packages are only required if you want to change the offline design.

## Run

Run [main.m](./main.m) to solve the SDP and store the results in [offline_design.mat](./offline_design.mat).

Done!

## How to customize?

Interested in trying out this method for your own nonlinear system? Easy! Follow these steps:

1. Implement your model based on [FalconModelT](./FalconModelT.m).
2. Change the line `model = <your_model_name>` in [main.m](./main.m) and add the correct function arguments.

That's it! Now you can run the SDP for your own system model :)

> :bulb: For non-holonomic systems, such as cars, it is necessary to enforce a minimum non-zero velocity.
