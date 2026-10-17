"""Road segmentation for the mat robot. No Webots imports, so the same code runs
in the controller, on a saved PNG, and later on an ESP32-CAM-sized frame.

Tuning on a saved frame (inside the container or on the host with opencv):
    python3 road_seg.py frame_0050.png [more.png ...] [--hlo 90 --hhi 135 --smax 85 --vlo 45 --vhi 130]
Writes <name>_seg.png (original | mask | overlay) and prints HSV statistics.
"""
import sys
import cv2
import numpy as np

# --- thresholds (OpenCV HSV: H 0-180, S 0-255, V 0-255) -----------------------
# Road is dark, bluish-grey: S roughly 25-80, V roughly 60-115 on the raw texture.
# Pavement is the same grey family but brighter (V ~150-190), so V is the real
# road/pavement discriminator. S excludes the coloured stuff (grass, yellow, red).
# Webots lighting shifts these numbers: re-tune on a frame the robot actually saved.
# Hue (measured on a real Webots frame): road 107-114, pavement ~109, brick ~170 (red),
# yellow signs ~21, grass/bushes ~58. A bluish-grey hue band removes everything coloured.
H_LO = 90
H_HI = 135
S_MAX = 85
V_LO = 45     # below this: black outlines around objects
V_HI = 130    # above this: pavement, lane paint, crossings
BLUR = 5      # odd; suppresses carpet-fibre speckle

_K_CLOSE = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))  # fills lane dashes
_K_OPEN = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))   # removes specks


def road_mask(bgr, s_max=None, v_lo=None, v_hi=None, h_lo=None, h_hi=None):
    """bgr: HxWx3 uint8. Returns HxW uint8, 255 = road, 0 = not road."""
    h_lo = H_LO if h_lo is None else h_lo
    h_hi = H_HI if h_hi is None else h_hi
    s_max = S_MAX if s_max is None else s_max
    v_lo = V_LO if v_lo is None else v_lo
    v_hi = V_HI if v_hi is None else v_hi
    bgr = np.ascontiguousarray(bgr)
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (BLUR, BLUR), 0), cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (h_lo, 0, v_lo), (h_hi, s_max, v_hi))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, _K_CLOSE)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, _K_OPEN)


def overlay(bgr, mask, colour=(255, 0, 255)):
    out = bgr.copy()
    sel = mask > 0
    out[sel] = (0.5 * out[sel] + 0.5 * np.array(colour)).astype(np.uint8)
    return out


def debug_montage(bgr, mask):
    """original | mask | overlay, side by side."""
    return np.hstack([bgr, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), overlay(bgr, mask)])


def _stats(bgr, mask):
    hsv = cv2.cvtColor(cv2.GaussianBlur(np.ascontiguousarray(bgr), (BLUR, BLUR), 0), cv2.COLOR_BGR2HSV)
    print(f"  road fraction: {(mask > 0).mean():.2f}")
    for name, sel in (("in mask ", mask > 0), ("outside ", mask == 0)):
        px = hsv[sel]
        if len(px):
            p = lambda c: np.percentile(px[:, c], [5, 50, 95]).astype(int)
            print(f"  {name} H p5/50/95 = {p(0)}   S = {p(1)}   V = {p(2)}")
    grey = hsv[(hsv[..., 1] < S_MAX) & (hsv[..., 0] >= H_LO) & (hsv[..., 0] <= H_HI)]
    low_s = grey[:, 2]
    if len(low_s) > 50:
        hist, _ = np.histogram(low_s, bins=np.arange(0, 261, 10))
        print("  V histogram of bluish-grey pixels (look for road hump, valley, pavement hump):")
        for b, c in zip(range(0, 260, 10), hist):
            if c:
                print(f"   {b:3d} {'#' * max(1, int(c / hist.max() * 40))}")


if __name__ == "__main__":
    args = sys.argv[1:]
    kw, files = {}, []
    while args:
        a = args.pop(0)
        if a in ("--smax", "--vlo", "--vhi", "--hlo", "--hhi"):
            kw[{"--smax": "s_max", "--vlo": "v_lo", "--vhi": "v_hi", "--hlo": "h_lo", "--hhi": "h_hi"}[a]] = int(args.pop(0))
        else:
            files.append(a)
    for f in files:
        img = cv2.imread(f)
        if img is None:
            print(f"cannot read {f}")
            continue
        m = road_mask(img, **kw)
        out = f.rsplit(".", 1)[0] + "_seg.png"
        cv2.imwrite(out, debug_montage(img, m))
        print(f"{f} -> {out}")
        _stats(img, m)
