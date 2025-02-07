#!/bin/bash

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"
DEPLOYMENT_DIR="$(realpath $SCRIPT_DIR/..)"
if [ -z "$1" ]
then
  echo "External packages path must be provided"
  exit 1
fi

export DTCVC_EXTERNAL_PACKAGES_PATH=$1
shift

$DEPLOYMENT_DIR/images/dtcvc-base/build.sh $@
$DEPLOYMENT_DIR/images/dtcvc-scorekeeper/build.sh $@
$DEPLOYMENT_DIR/images/dtcvc-agent/build.sh $@
$DEPLOYMENT_DIR/images/dtcvc-simulator/build.sh $@