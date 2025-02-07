#!/bin/bash

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"
REPO_DIR="$(realpath $SCRIPT_DIR/../../..)"
IMAGE_LABEL="${IMAGE_REGISTRY}dtcvc-base:${IMAGE_TAG:-latest}"

if [ -z "$DTCVC_EXTERNAL_PACKAGES_PATH" ]
then
  echo "External packages path must be provided"
  exit 1
fi

set -x
docker build $@ -f $SCRIPT_DIR/Dockerfile -t $IMAGE_LABEL --build-context "external-packages=$DTCVC_EXTERNAL_PACKAGES_PATH" $REPO_DIR/dtcvc
