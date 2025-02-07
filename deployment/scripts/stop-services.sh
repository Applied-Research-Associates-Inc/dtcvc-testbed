#!/bin/bash

docker service rm dtcvc-discovery
docker network rm dtcvc-network
docker stop dtcvc-nfs -t 10 && docker rm dtcvc-nfs
