#!/bin/bash

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"
REPO_DIR="$(realpath $SCRIPT_DIR/../../..)"
IMAGE_LABEL="${IMAGE_REGISTRY}dtcvc-agent:${IMAGE_TAG:-latest}"
BASE_IMAGE="${IMAGE_REGISTRY}dtcvc-base:${IMAGE_TAG:-latest}"

docker build $1 -f $SCRIPT_DIR/Dockerfile --build-arg="BASE_IMAGE=$BASE_IMAGE" -t $IMAGE_LABEL $REPO_DIR/dtcvc-competitor
