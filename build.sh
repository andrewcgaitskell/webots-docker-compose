#!/bin/bash
xhost +local:docker
docker compose up -d --build --force-recreate
