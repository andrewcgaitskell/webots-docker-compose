from controller import Robot
import numpy as np

robot = Robot()
dt = int(robot.getBasicTimeStep())
cam = robot.getDevice("camera"); cam.enable(dt)
lm = robot.getDevice("left_motor");  lm.setPosition(float('inf'))
rm = robot.getDevice("right_motor"); rm.setPosition(float('inf'))

SPEED = 4.0
n = 0
while robot.step(dt) != -1:
    img = np.frombuffer(cam.getImage(), np.uint8).reshape(cam.getHeight(), cam.getWidth(), 4)
    bgr = img[:, :, :3]          # same frame an ESP32-CAM would give you (160x120)
    # TODO: road segmentation + steering goes here
    lm.setVelocity(SPEED); rm.setVelocity(SPEED)
    n += 1
    if n == 50:
        cam.saveImage("/tmp/cam_test.png", 100)   # check what the robot sees
