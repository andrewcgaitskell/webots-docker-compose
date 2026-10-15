FROM cyberbotics/webots:R2025a-ubuntu22.04
RUN apt-get update && apt-get install -y python3 python3-pip python3-numpy python3-opencv \
    && rm -rf /var/lib/apt/lists/*

