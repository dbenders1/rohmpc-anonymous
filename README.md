# rohmpc

[![License](https://img.shields.io/badge/license-MIT-blue)](https://opensource.org/licenses/MIT)



## ICRA 2026 submission
The reference implementation of our ICRA 2026 submission:

**From Data to Safe Mobile Robot Navigation: An Efficient, Modular, and Reproducible Robust MPC Design Pipeline**



## Summary
In this work, we propose the following pipeline for the synthesis of robust (output-feedback) MPC schemes as visualized in [this figure](./media/pipeline.png) (does not load correctly on Anonymous GitHub).

This pipeline is:

:stopwatch: **Efficient** - Within 2 hours, you can have a robust output-feedback MPC scheme running with empirical guarantees on robust constraint satisfaction and recursive feasibility!

:jigsaw: **Modular** - The design pipeline allows to integrate your own dynamic model and adjust other system properties to customize the setup for your own application!

:repeat: **Reproducible** - All simulations are deterministic will give the same results every run. You missed some data? No problem, you can re-run the pipeline at any time to regenerate the results!

Visit the following links to see how each of the steps in the pipeline is implemented.
1. [Uncertainty quantification](./src/uq.md)
2. [Robust and terminal ingredients design](./src/robust_term_design.md)
3. [Tightening calibration](./src/tightening_calib.md)
4. [ROHMPC deployment and analysis](./src/rohmpc.md)

[This animation](./media/rohmpc.gif) shows the ROHMPC framework operating in a Gazebo simulation.

We challenge you to reproduce this result in our paper! :wink:



## How to get started?
**Interested in reproducing our results and extending them for your own work? No problem!**

This repository contains the code and documentation to reproduce the presented Gazebo simulations in the paper. We also provide a simple simulation node for control over the simulated quadrotor dynamics.

To get started, clone this repository and follow the instructions in the [src README](./src/README.md).



## Acknowledgement
The results in this repository are based on an adjusted version of the [Agilicious](https://github.com/uzh-rpg/agilicious) stack.
