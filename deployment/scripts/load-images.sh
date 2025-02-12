#!/bin/bash

cat $1 | pv | ssh $2 -C 'docker load'