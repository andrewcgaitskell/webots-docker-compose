#!/bin/bash

# Stop Webots
docker compose down

# Disable X server connections
xhost -local:root > /dev/null 2>&1

