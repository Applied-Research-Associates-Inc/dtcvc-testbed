#!/bin/bash

if [ ! -s "$1" ]
then
  echo "Path to compose file must be provided"
  exit 1
fi

export NFS_ADDR=$(hostname -I | awk '{print $1}')
export DTCVC_DISCOVERY_ADDRESS=$(docker service inspect dtcvc-discovery --format '{{range .Endpoint.VirtualIPs}}{{.Addr}}{{end}}' | sed 's|/.*||')

compose_file=$1
filename="${1##*/}"
extension="${filename##*.}"
basename="${filename%.*}"
shift

docker stack deploy --resolve-image never -c <(docker compose -f "$compose_file" config | sed -E '/^name:/d;s/published: "([[:digit:]]+)"/published: \1/g') $basename $@