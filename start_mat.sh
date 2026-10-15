#!/bin/bash
xhost +local:docker
docker compose -f docker-mat.yml up -d
