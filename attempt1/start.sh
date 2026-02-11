#!/bin/bash

# Enable X server connections
xhost +local:root > /dev/null 2>&1

# Start Webots
docker compose up -d
