#!/bin/bash

docker save dtcvc-base:latest dtcvc-scorekeeper:latest dtcvc-agent:latest dtcvc-simulator:latest | gzip | pv > $1