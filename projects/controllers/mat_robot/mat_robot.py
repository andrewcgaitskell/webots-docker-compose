import os
import cv2
import numpy as np
from controller import Robot
from road_seg import road_mask, debug_montage

HERE = os.path.dirname(os.path.abspath(__file__))   # ./projects is volume-mounted, so files appear on the host
SAVE_STEPS = (50, 150, 300, 450)                     # frames to save for threshold tuning
LOG_EVERY = 25

robot = Robot()
dt = int(robot.getBasicTimeStep())
cam = robot.getDevice("camera"); cam.enable(dt)
disp = robot.getDevice("seg_display")      # shown via the Overlays menu in Webots
lm = robot.getDevice("left_motor");  lm.setPosition(float('inf'))
rm = robot.getDevice("right_motor"); rm.setPosition(float('inf'))

SPEED = 4.0
n = 0
while robot.step(dt) != -1:
    img = np.frombuffer(cam.getImage(), np.uint8).reshape(cam.getHeight(), cam.getWidth(), 4)
    bgr = np.ascontiguousarray(img[:, :, :3])   # same frame an ESP32-CAM would give you (160x120)
    mask = road_mask(bgr)
    # TODO: steering from the mask goes here. For now: drive straight, observe only.
    lm.setVelocity(SPEED); rm.setVelocity(SPEED)
    n += 1
    ref = disp.imageNew(cv2.cvtColor(debug_montage(bgr, mask), cv2.COLOR_BGR2BGRA).tobytes(),
                        disp.BGRA, 3 * cam.getWidth(), cam.getHeight())
    disp.imagePaste(ref, 0, 0, False)
    disp.imageDelete(ref)
    if n % LOG_EVERY == 0:
        print(f"step {n}: road fraction {(mask > 0).mean():.2f}")
    if n in SAVE_STEPS:
        cv2.imwrite(os.path.join(HERE, f"frame_{n:04d}.png"), bgr)                    # raw: tune on this
        cv2.imwrite(os.path.join(HERE, f"frame_{n:04d}_seg.png"), debug_montage(bgr, mask))
