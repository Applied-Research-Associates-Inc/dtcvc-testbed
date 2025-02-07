#!/bin/bash

# set -x

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"
DEPLOYMENT_DIR="$(realpath $SCRIPT_DIR/..)"

# Run a containerized NFSv4 server on the host (not a part of the swarm) for sharing configs and collecting outputs
docker run --name dtcvc-nfs \
    --privileged \
    --security-opt apparmor=erichough-nfs \
    -e NFS_DISABLE_VERSION_3=true \
    -e NFS_LOG_LEVEL=DEBUG \
    -e NFS_EXPORT_0='/data *(rw,sync,no_subtree_check,no_root_squash,fsid=0)' \
    -e NFS_EXPORT_1='/data/config/testbed *(ro,sync,nohide,no_subtree_check,no_root_squash,fsid=1)' \
    -e NFS_EXPORT_2='/data/config/agent *(ro,sync,nohide,no_subtree_check,no_root_squash,fsid=2)' \
    -e NFS_EXPORT_3='/data/output/agent *(rw,sync,nohide,no_subtree_check,no_root_squash,fsid=3)' \
    -e NFS_EXPORT_4='/data/output/testbed *(rw,sync,nohide,no_subtree_check,no_root_squash,fsid=4)' \
    -v "${DTCVC_DATA_PATH:-$DEPLOYMENT_DIR/data}:/data" \
    -p 2049:2049   -p 2049:2049/udp   \
    --detach \
    erichough/nfs-server:latest

# Create an attachable overlay network for the swarm deployment
docker network create -d overlay --attachable dtcvc-network

# Create a ROS2 discovery server attached to the swarm network
docker service create --name dtcvc-discovery --network dtcvc-network --constraint node.role==manager osrf/ros:jazzy-simulation fastdds discovery --server-id 0 -l 0.0.0.0 -p ${DISCOVERY_SERVER_PORT:-11811}