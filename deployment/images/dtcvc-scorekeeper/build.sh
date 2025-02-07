#!/bin/bash

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"
REPO_DIR="$(realpath $SCRIPT_DIR/../../..)"
IMAGE_LABEL="${IMAGE_REGISTRY}dtcvc-scorekeeper:${IMAGE_TAG:-latest}"
BASE_IMAGE="dtcvc-base:${IMAGE_TAG:-latest}"

docker build $@ -f $SCRIPT_DIR/Dockerfile --build-arg="BASE_IMAGE=$BASE_IMAGE" --build-context "dtcvc=$REPO_DIR/dtcvc" -t $IMAGE_LABEL $SCRIPT_DIR
