import os
import cv2
import numpy as np
from controller import Robot
from road_seg import road_mask
from road_score import analyse, montage

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ.get("CAPTURE_DIR", HERE)           # docker_mat.yml maps CAPTURE_DIR to a host folder
os.makedirs(OUT, exist_ok=True)
SAVE_STEPS = range(50, 601, 50)                      # frames to save for threshold tuning
LOG_EVERY = 25                                       # console line
CSV_EVERY = 5                                        # row in bands.csv

robot = Robot()
dt = int(robot.getBasicTimeStep())
cam = robot.getDevice("camera"); cam.enable(dt)
disp = robot.getDevice("seg_display")      # 960 x 240, shown via the Overlays menu in Webots
lm = robot.getDevice("left_motor");  lm.setPosition(float('inf'))
rm = robot.getDevice("right_motor"); rm.setPosition(float('inf'))

csv = open(os.path.join(OUT, "bands.csv"), "w")
csv.write("step,lost,steer,aim_cm,lat_cm,heading_deg,junction,"
          + ",".join(f"B{i}_score,B{i}_off_cm,B{i}_ratio" for i in range(1, 5)) + "\n")

SPEED = 4.0
n = 0
while robot.step(dt) != -1:
    img = np.frombuffer(cam.getImage(), np.uint8).reshape(cam.getHeight(), cam.getWidth(), 4)
    bgr = np.ascontiguousarray(img[:, :, :3])   # same frame an ESP32-CAM would give you (160x120)
    mask = road_mask(bgr)
    res = analyse(mask)                          # per-band offsets, scores, steer value, junction flag
    # TODO: use res['steer'] (-1 left .. +1 right, positive = road is to the right) to drive. For now: straight.
    lm.setVelocity(SPEED); rm.setVelocity(SPEED)
    n += 1

    view = montage(bgr, mask, res)               # original | road as traced | overlay with scores
    ref = disp.imageNew(cv2.cvtColor(view, cv2.COLOR_BGR2BGRA).tobytes(), disp.BGRA, view.shape[1], view.shape[0])
    disp.imagePaste(ref, 0, 0, False)
    disp.imageDelete(ref)

    if n % CSV_EVERY == 0:
        b = res['bands']
        csv.write(f"{n},{int(res['lost'])},{res['steer']:.3f},{res['aim_cm']:.2f},{res['lat_cm']:.2f},"
                  f"{np.degrees(res['heading']):.1f},{'+'.join(res['junction'])},"
                  + ",".join(f"{x['score']:.2f},{x['off_cm']:.2f},{x['ratio']:.2f}" for x in b) + "\n")
        csv.flush()
    if n % LOG_EVERY == 0:
        cells = " ".join(f"{x['name']} {x['score']:.2f}/{x['off_cm']:+.1f}" for x in res['bands'])
        print(f"step {n}: {cells} | steer {res['steer']:+.2f}"
              + (" ROAD LOST" if res['lost'] else "") + (f" junction {res['junction']}" if res['junction'] else ""))
    if n in SAVE_STEPS:
        cv2.imwrite(os.path.join(OUT, f"frame_{n:04d}.png"), bgr)                 # raw: tune on this
        cv2.imwrite(os.path.join(OUT, f"frame_{n:04d}_seg.png"), view)
