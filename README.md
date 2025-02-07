# DARPA Triage Challenge - Virtual Competition - Public Repository

This repository contains resources and documentation for DTC-VC competitors, such as container images and local deployment scripts, a sample competitor container image, and the automated scoring library and service.

The sample competitor container implementation can be found in `dtcvc-competitor`. `dtcvc-competitor/ros` contains the Python ROS2 scripts implementing the competitor and `dtcvc-competitor/deployment` contains the `Dockerfile` and scripts for building the competitor container. The sample competitor container is provided for reference only. Competitors are not required to base their implementation on the sample, however they must ensure any custom container they submit behaves similarly to the sample.

## Minimum Requirements

- A Linux host with recent kernel/distro (Ubuntu 22.04, etc.)
- NVIDIA GPU and latest Linux drivers installed (required for GPU access in containers)
- Docker

## Prepare container images

*Using prebuilt container images*

- Download DTC-VC container images tarball (named `dtcvc-images-rYYYYMMDD.tar`), for this example it's path is `~/path/to/dtcvc-images.tar`, replace with the actual location accordingly.
- Load images by issuing the following:
  - `docker load -i ~path/to/dtcvc-images.tar`
  - Or if the tarball is compressed: `gunzip -c path/to/dtcvc-images.tgz | docker load`

*Building container images locally*

- Download and extract the DTC-VC simulator package (named `dtcvc-simulator-rYYYYMMDD.zip`) into a local folder, such as `~/path/to/dtcvc-simulator`
- Then either:
    - Use the `build.sh` script to only build the images:
      ```bash
      ./build.sh ~/path/to/dtcvc-simulator
      ```
    - Use the `deploy.sh` script with the `--build` argument to invoke `build.sh` implicitly before deploying (see below for more details):
      ```bash
      ./deploy.sh --build ~/path/to/dtcvc-simulator up
      ```

## Deployment variations

> NOTE:
> The competition is intended to be ran in a full Linux environment. Use of the `--windows` flag is only experimental.

There are currently two deployment variations. Each designed for Linux deployment with a Windows WSL alternative.
Deployments can be done with raw "docker compose", however it is easier to use the pre-made `./deploy.sh` which itself extends off of the `docker compose` command and can be passed in every argument that `docker compose` can intake.

The script can either make temporary deployments or persist deployment settings with a .env file located at "dtcvc/deployment/config". 

> NOTE: 
> It is not recommended to modify the '.env.example' file as the script uses this to help generate new .env files. 

In terms of extensions from `docker compose`, the script has several new arguments and overrides of pre-existing ones. To see explanations of these arguments, pass in the `--help` flag into the script. For docker compose specific explanations, pass in the `--more-help` flag into the script.

### Basic deployment arguments

The following documentation outlines common commands and arguments that covers the needs of post competitors.

#### General commands:

- `up`: Brings containers up.
  - Example: `./deploy.sh [DEPLOY ARGUMENTS] up [COMPOSE OPTIONS]`
- `down`: Takes containers down.
  - Example: `./deploy.sh [DEPLOY ARGUMENTS] down [COMPOSE OPTIONS]`
- `exec`: Enters into container terminal.
  - Example: `./deploy.sh exec [CONTAINER NAME] /bin/bash`
  - 
#### General deploy arguments:
- `--scenario scenarioNNN.yaml`: Specify the built-in scenario file for the simulator to run, where `NNN` ranges from `001` to `006`, if not specified then defaults to `scenario001.yaml`.
- `--nowrite`: Passes in settings for a single deployment only, does not write to the .env.
- `--simrec`: Deploy competition in 'Simulator Redording Mode' to generate a rosbag recording of the simulation scenario which can be played back more efficiently on systems with lower performance GPUs.
- `--nosim`: Deploy competition in 'No Simulator Mode', which requires much less GPU usage.
  - `-pn` or `--playback-name`: Define filename to use for 'No Simulator Mode' playback.
- `-rn` or `--recording-name`: Define filename of competition recording.
- 
#### General docker compose options:
- `--build`: Builds relevant images before deployment.
- `-d` or `--detach`: Detaches deployment consoles from the execution console.
- `--force-recreate`: Forcefully recreates containers.

### Normal Deployment

These deployments bring up the competitor container, scorekeeper container, and simulator container. These are the normal competition deployments, and are very GPU heavy.

```bash
./deploy.sh up
```

Or, to run a specific scenario instead of the default (see `dtcvc/deployment/config/scenario.example.yaml`):
```bash
./deploy.sh --scenario 'scenario.yaml' up
```

Log files and recordings will be placed in `output/logs` and `output/recordings` respectively.

### Simulation Recording Deployment

This deployment brings up only the scorekeeper and simulator containers. These deployments are only for recording a simulation scenario for future playback in "No Simulation" deployments. They are very GPU heavy.

```bash
./deploy.sh --simrec --recording-name recording1.bag up
```

The recording will be placed in `output/scenario_recordings`.

### No Simulation Deployment

This deployment only brings up the scorekeeper and competitor containers, as it plays back a recorded simulation from a rosbag for those with insufficient hardware to run the simulator. There is no windows version for this type of deployment as there is no simulation. These deployments are very RAM heavy, but light on GPUs.

```bash
./deploy.sh --nosim --playback-name recording1.bag up
```

### Deploy with Foxglove for Debugging

Foxglove provides a web based UI for ROS2 for realtime visualization and debugging. To enable the Foxglove server, specify the `debug` profile:

```bash
./deploy.sh --profile debug up
```

This will launch the Foxglove server at `http://localhost:8080`. Once the Foxglove client is loaded in the browser, use a websocket connection to the Foxglove ROS bridge running on the host: `ws://localhost:8765`.

### Scorekeeper Output

Final reports are generated from within the scorekeeper container and can be found in the following directory: `public/output/logs/`. Additionally, final reports are mounted in the volume directory: `/opt/logs/`. A single report is generated per run of the simulation.

Final reports have the following name structure: `final_score_report_{timestamp_in_ns}`

The final score can be found at the top of the final report. Casualty assessments are mappings of casualty IDs to reports, an aggregate, and a score and are listed after the total score.
