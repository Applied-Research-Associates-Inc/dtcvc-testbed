# DARPA Triage Challenge - Virtual Competition - Public Repository

This repository contains resources and documentation for DTC-VC competitors, such as container images and local deployment scripts for the DTC-VC Testbed, a sample competitor container image, and the automated scoring library and service.

## Minimum Requirements

- A Linux host with recent kernel/distro (Ubuntu 22.04, etc.)
- NVIDIA GPU and latest Linux drivers installed (required for GPU access in containers)
- The latest version of Docker
- Multiple hosts are recommended for best performance

## Getting Started

### Setup Docker Swarm

This process involves a lot of tricky steps, so please ensure you read them carefully. **Copy/Pasting all the steps will not work, there is manual configuration**.

Please reference the official Docker Swarm [documentation](https://docs.docker.com/engine/swarm/) and [tutorial](https://docs.docker.com/engine/swarm/swarm-tutorial/) for more details. The following only covers configuration required for for the DTC-VC testbed and the essential steps needed to initialize a swarm.

#### Configure Docker Daemon On All Node

All hosts participating in the swarm with GPUs require special configured to allow GPU access from containers launched in the swarm. Open and modify the Docker Daemon configuration on each node (usually located at `/etc/docker/daemon.json`) to include the line `"default-runtime": "nvidia"`. This ensures that all containers use the Nvidia container runtime by default since Docker Swarm does not support configuring the container runtime. See this [link](https://gist.github.com/coltonbh/374c415517dbeb4a6aa92f462b9eb287) for more details. For example (your `daemon.json` file may be different):

```json
{
   "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    }
}
```

When the configuration has been updated, run the command below to restart Docker:

```bash
sudo service docker restart
```

#### Configure Manager Node

**Configure Kernel Modules**

The containerized NFS server used in swarm deployments requires loading NFS kernel modules:

```bash
sudo modprobe nfs nfsd
```

To persist the module configuration add the following lines to `/etc/modules-load.d/modules.conf`:

```
nfs
nfsd
```

**AppArmor Profile (optional)**

If your environment uses AppArmor then additional configuration is required to allow the containerized NFS server used in swarm deployments to run in privileged mode. First, install the required packages:

```bash
sudo apt-get install apparmor-utils lxc
```

Next, run the following commands to :

```bash
cd path/to/dtcvc/deployment
sudo apparmor_parser -r -W config/erichough-nfs.apparmor
sudo modprobe nfs
sudo modprobe nfsd
```

To persist the AppArmor configuration do the following:

```bash
sudo cp erichough-nfs.apparmor /etc/apparmor.d/
sudo systemctl reload apparmor
```

#### Initialize Swarm

First, at least one host must be initialized as a manager to create the swarm. Be sure to specify the IP address of an interface on the manager which can communicate with all other hosts in the swarm:

```bash
docker swarm init --advertise-addr <MANAGER-IP>
```

Then, on all other hosts, run `docker swarm join` with the arguments shown in the output of the swarm init command. For example:

```bash
docker swarm join \
--token SWMTKN-1-49nj1cmql0jkz5s954yi3oex3nedyz0fb0xx14ie39trti4wxv-8vxv8rssmk743ojnwacrr2e7c \
192.168.99.100:2377
```

Once all hosts have been added to the swarm, on a manager node run the following to see all the nodes in the swarm:

```bash
docker node ls
```

#### Assign Labels to Swarm Nodes

Node labels can be used to constrain the swarm nodes to which deployed services are assigned. For example, to add a `gpu` label to a node, run the following command on the manager node:

```bash
docker node update --label-add gpu=rtx4090 some-node-hostname
```

To remove a label:

```bash
docker node update --label-rm gpu some-node-hostname
```

### Deploy Swarm Support Services

DTC-VC testbed deployments require some additional services running on the manager node:
- An overlay network for the testbed
- A containerized NFS server to allow services on other hosts to access the `deployment/data` folder on the manager
- A ROS2 discovery server to allow ROS2 nodes to pass messages (the default discovery mechanism relies on UDP broadcast for dicovery which swarm overlay networks do not support)

All of these services can be started with the command:

```bash
./deployment/scripts/swarm-start-services.sh
```

And if you need to stop, or reset it, run the command:

```bash
./deployment/scripts/swarm-stop-services.sh
```

### Prepare Container Images

Container images must be built or loaded onto each host participating in the swarm.

**Using prebuilt container images**

- Download the DTC-VC container images package (named `dtcvc-images-rYYYYMMDD.tgz`), for this example it's path is `~/path/to/dtcvc-images.tgz`, replace with the actual location accordingly.
- Load images by issuing the following:
  - `docker load -i ~/path/to/dtcvc-images.tgz`

**Building container images locally**

- Download and extract the DTC-VC simulator package (named `dtcvc-simulator-rYYYYMMDD.zip`) into a local folder, such as `~/path/to/dtcvc-simulator`
- Use `deployment/scripts/build-images.sh` to build the images:
  ```bash
  ./deployment/scripts/build-images.sh ~/path/to/dtcvc-simulator/Carla-0.10.0-Linux-Development
  ```

### Deploy the Testbed

#### Configure the scenario

Each deployment of the DTC-VC testbed will perform a single run with the configured scenario. The default scenario configuration file is located at `deployment/data/config/testbed/scenario.yml`. Modify this file to change the simulation runtime or the loaded environment.

#### Configure the deployment

The swarm deployment is configured using [compose files](https://docs.docker.com/reference/compose-file/), two examples are provided:
- `deployment/swarm/single-agent.yml`: A deployment with a single simulator and agent
- `deployment/swarm/solely-sim.yml`: A deployment with a single simulator and no agents, useful for testing the simulator

**Set placement constraints**

Swarm [placement constraints](https://docs.docker.com/reference/compose-file/deploy/#placement) are used to control on which nodes services are allocated. If no constraints are specified for a service then it may be allocated on any available node in the swarm. For example, constraints can be used to ensure that services which require a GPU are only allocated to hosts with a GPU. To configure the simulator service in the `single-agent.yml` deployment add a contraint on the `gpu` label (see above for examples of assigning labels to nodes):

```yml
  simulator1:
    ...
    deploy:
      placement:
        constraints:
          - node.labels.gpu == rtx4090
    ...
```

To constrain a service to the manager node use:

```yml
          - node.role == manager
```

To constrain a service to any node but the manager use:

```yml
          - node.role != manager
```

To constrain a service to a node with a specific hostname:

```yml
          - node.hostname == isr-ace-tribox
```

#### Perform a deployment

The `deployment/scripts/deploy.sh` script encapsulates the `docker stack deploy` command and sets required environment variables used in the swarm deployment compose files. For example, to deploy the `single-agent.yml` compose file:

```bash
./swarm-deploy.sh ./swarm/single-agent.yml
```

This will immediately exit and should not produce any obvious crash or errors.

To check the status of the deployed testbed services use the `docker stack ps` command. Add the `--no-trunc` argument to show full error messages:

```bash
docker stack ps single-agent
```

Use the `docker service logs` command to see the logs of the deployed testbed services. Add a `-f` at the end to make it stream the output:

```bash
docker service logs single-agent_scorekeeper
docker service logs single-agent_simulator1
docker service logs single-agent_agent1
```

The testbed services will automatically start running the configured scenario once all services are ready. Once the scenario finishes the testbed services will all shutdown, but there is no way to automatically cleanup the deployment. To stop and remove the deployment, use the `docker stack rm` command:

```bash
docker stack rm single-agent
```

**Using the DTC Testing Container**

The DTC Testing Container provides utilities for interacting with the simulator during development. [See the documentation for more details](./docs/using_testing_container.md).

### Scorekeeper Output

Final reports generated by the scorekeeper service can be found in the testbed log output directory: `deployment/data/output/testbed/logs/final_score_report_{timestamp_in_ns}.json`. A single report is generated per run of the simulation.

The final score can be found at the top of the final report. Casualty assessments are mappings of casualty IDs to reports, an aggregate, and a score and are listed after the total score.

## Repository Organization

- `docs`: Additional documentation
- `deployment`: Container images, configuration, and scripts used for deployments
- `dtcvc`: Python and ROS2 packages used by the DTC-VC testbed
  - `dtcvc/lib/dtcvc-scoring`: The Python library used for scoring triage reports
  - `dtcvc/ros/src`: ROS2 packages used by the testbed
- `dtcvc-competitor`: Reference implementation for a competitor agent demonstrating ROS2 topics used in a deployment and triage report submission
  - A sample Dockerfile for running the reference implementation can be found in `deployment/images/dtcvc-agent`
  - The sample competitor container is provided for reference only. Competitors are not required to base their implementation on the sample, however they must ensure any custom container they submit behaves similarly to the sample.