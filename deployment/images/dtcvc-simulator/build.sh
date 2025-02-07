#!/bin/bash

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"
IMAGE_LABEL="${IMAGE_REGISTRY}dtcvc-simulator:${IMAGE_TAG:-latest}"

DTCVC_EXTERNAL_PACKAGES_PATH=${DTCVC_EXTERNAL_PACKAGES_PATH:-$1}

if [ -z "$DTCVC_EXTERNAL_PACKAGES_PATH" ]
then
  echo "External packages path must be provided"
  exit 1
fi

# Build the image, pipe in the Dockerfile to disable the default build context
docker build $@ -t $IMAGE_LABEL --build-context "external-packages=$DTCVC_EXTERNAL_PACKAGES_PATH" - < $SCRIPT_DIR/Dockerfile
