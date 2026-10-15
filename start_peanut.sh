#!/bin/bash
xhost +local:docker
docker compose -f docker_peanut.yml up -d
