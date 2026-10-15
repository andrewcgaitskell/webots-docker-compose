#!/bin/bash
xhost +local:docker
docker compose -f docker_mat.yml up -d --build
