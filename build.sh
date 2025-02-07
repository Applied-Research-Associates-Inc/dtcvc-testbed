#!/bin/bash

set -e
set -x

SCRIPT_DIR="$(realpath $(dirname $BASH_SOURCE[0]))"

if [ -z "$1" ]
then
  echo "External packages path must be provided"
  exit 1
fi

export DTCVC_EXTERNAL_PACKAGES_PATH=$1
shift

$SCRIPT_DIR/dtcvc/deployment/dtcvc-simulator/build.sh $@
$SCRIPT_DIR/dtcvc/deployment/dtcvc-base/build.sh $@
$SCRIPT_DIR/dtcvc/deployment/dtcvc-scorekeeper/build.sh $@
$SCRIPT_DIR/dtcvc-competitor/deployment/dtcvc-competitor/build.sh $@