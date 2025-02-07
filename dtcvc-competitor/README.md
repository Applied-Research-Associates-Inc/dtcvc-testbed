# DARPA Triage Challenge - Virtual Competition - Competitor Reference Sample

This repository provides a reference sample for competitor submissions to the DTC-VC.

The Dockerfile located at "deployment/dtcvc-competitor" shows an example installation of necessary libraries/packages using `pip` and `apt` package managers. The use of conda is not recommended and will cause unexpected behavior. This is because ROS2 requires python packages to be installed within the default system directory, while conda/mamba use their own environment directories.