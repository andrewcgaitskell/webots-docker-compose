# Reference

https://cyberbotics.com/doc/guide/installation-procedure#installing-the-docker-image

# Copilot Prompt

log winded - convert the following to a compose approach - Without GPU Acceleration

To run Webots with a graphical user interface in a docker container, you need to enable connections to the X server before starting the docker container:

xhost +local:root > /dev/null 2>&1

    Note: If you need to disable connections to the X server, you can do it with the following command: xhost -local:root > /dev/null 2>&1.

You can then start the container with the following command:

docker run -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest

Or if you want to directly launch Webots:

docker run -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest webots

# Webots Docker Compose (No GPU)

## Quick Start

```bash
./start.sh
```

## Stop

```bash
./stop.sh
```

## What It Does

- Enables X server connections (`xhost +local:root`)
- Starts Webots in Docker container
- Maps display for GUI
- On stop, disables X server connections (`xhost -local:root`)

## Manual Commands

```bash
# Start
xhost +local:root > /dev/null 2>&1
docker-compose up

# Stop
docker-compose down
xhost -local:root > /dev/null 2>&1
```
