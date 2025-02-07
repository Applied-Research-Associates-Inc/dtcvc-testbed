#!/bin/bash

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"
DEPLOYMENT_DIR="$(realpath $SCRIPT_DIR/../..)"
IMAGE_LABEL="dtcvc-scorekeeper:${IMAGE_TAG:-latest}"
BASE_IMAGE="dtcvc-base:${IMAGE_TAG:-latest}"

docker build $@ -f $SCRIPT_DIR/Dockerfile --build-arg="BASE_IMAGE=$BASE_IMAGE" -t $IMAGE_LABEL $DEPLOYMENT_DIR
