#!/bin/bash

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"
IMAGE_LABEL="dtcvc-simulator:${IMAGE_TAG:-latest}"

if [ -z "$DTCVC_EXTERNAL_PACKAGES_PATH" ]
then
  echo "External packages path must be provided"
  exit 1
fi

docker build $@ -t $IMAGE_LABEL --build-context "external-packages=$DTCVC_EXTERNAL_PACKAGES_PATH" - < $SCRIPT_DIR/Dockerfile
