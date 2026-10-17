"""Score the road mask in distance bands and turn it into numbers a steering algorithm can use.
No Webots imports, so it runs in the controller and on saved frames:
    python3 road_score.py frame_0050.png [more.png ...]     -> <name>_score.png + a printed table

Idea: the camera geometry is known, so every image row corresponds to a known ground distance and a
known size in cm. That gives, per band:
  offset_cm  where the road centre is relative to the camera axis (+ = road to the right = steer right)
  ratio      visible road width / expected road width (about 1 on a plain road, >1.3 widening/junction)
  score      0..1 confidence (coverage x width plausibility x single-blob)
The dashed centre line is NOT used (the mask already fills it). The road is one way, so when the
road splits we follow the part that continues straight up from the bottom of the frame.
"""
import sys
import cv2
import numpy as np

# ---- camera + road model: edit these to match the robot --------------------------------------
CAM_H = 0.05            # camera height above ground (m)
TILT = 0.6632           # angle below horizontal (rad), 38 deg
HFOV = 1.13             # horizontal field of view (rad)
W, H = 160, 120
ROAD_W_CM = 6.0         # road width (sim mat measures 6.0-6.4 cm)
BAND_EDGES_CM = (2.0, 6.0, 10.0, 15.0, 22.0)   # ground distance ahead of the camera
# ---- scoring / steering knobs (untuned starting values) --------------------------------------
GAP_FRAC = 0.3          # gaps narrower than this x road width inside a row are merged (centre-line dashes)
WIDE_RATIO = 1.3        # visible width / expected above this = road widening (junction candidate)
MIN_SCORE = 0.3         # bands below this are ignored by the steering fit
MIN_COV = 0.6           # ...and so are bands where less than this fraction of rows was traced
AIM_CM = 10.0           # distance ahead of the camera of the steering aim point
AIM_SCALE_CM = 3.0      # aim point this far off the camera axis = full steer (+-1)


def _geometry():
    f = (W / 2) / np.tan(HFOV / 2)
    d = np.full(H, np.nan)
    cm_px = np.full(H, np.nan)
    for r in range(H):
        vy = -(r + 0.5 - H / 2)
        dz = -f * np.sin(TILT) + vy * np.cos(TILT)
        dx = f * np.cos(TILT) + vy * np.sin(TILT)
        if dz < -1e-9:                       # row looks at the ground (below the horizon)
            t = -CAM_H / dz
            d[r] = dx * t * 100              # ground distance ahead of the camera (cm)
            cm_px[r] = t * 100               # ground cm per image pixel along this row
    return d, cm_px


D_CM, CM_PER_PX = _geometry()
EXP_W_PX = ROAD_W_CM / CM_PER_PX             # expected road width in pixels, per row
BANDS = [np.where((D_CM >= lo) & (D_CM < hi))[0]
         for lo, hi in zip(BAND_EDGES_CM[:-1], BAND_EDGES_CM[1:])]


def bottom_region(mask):
    """Keep only the road region that touches the bottom of the frame (drops stray blobs)."""
    n, lab, _, _ = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), connectivity=8)
    if n <= 1:
        return np.zeros_like(mask)
    counts = np.bincount(lab[H - 12:, :].ravel(), minlength=n)
    counts[0] = 0
    if counts.max() == 0:                    # nothing at the robot's feet: not on the road
        return np.zeros_like(mask)
    return ((lab == counts.argmax()).astype(np.uint8)) * 255


def _runs(row, r, min_len=3):
    """Runs of road pixels in one image row; narrow gaps (centre-line dashes) are bridged."""
    x = np.concatenate(([0], (row > 0).astype(np.int8), [0]))
    dd = np.diff(x)
    runs = [(a, b) for a, b in zip(np.where(dd == 1)[0], np.where(dd == -1)[0]) if b - a >= min_len]
    merged = []
    for a, b in runs:
        if merged and a - merged[-1][1] <= GAP_FRAC * EXP_W_PX[r]:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    return merged


def _trace(mask):
    """Follow the road upward from the bottom row; at each row take the run that overlaps the one below."""
    L = np.full(H, -1)
    R = np.full(H, -1)
    nruns = np.zeros(H, int)
    prev = None
    for r in range(H - 1, int(BANDS[-1][0]) - 1, -1):
        runs = _runs(mask[r], r)
        nruns[r] = len(runs)
        if not runs:
            break
        if prev is None:
            pick = max(runs, key=lambda a: a[1] - a[0])
        else:
            ov = [min(b, prev[1]) - max(a, prev[0]) for a, b in runs]
            if max(ov) <= 0:
                break                        # road ends / lost
            pick = runs[int(np.argmax(ov))]
        L[r], R[r] = pick
        prev = pick
    return L, R, nruns


def _row_centre(l, r, row):
    """Road centre (px) for one row. If one edge is cut off by the image border, use the known road width."""
    exp = EXP_W_PX[row]
    cut_l, cut_r = l <= 1, r >= W - 1
    if not cut_l and not cut_r:
        return (l + r) / 2, 'both'
    if cut_l and not cut_r:
        return r - exp / 2, 'right'
    if cut_r and not cut_l:
        return l + exp / 2, 'left'
    return W / 2, 'none'                     # road fills the row: centre unknown, assume centred


def analyse(mask):
    """mask: HxW uint8 from road_seg.road_mask. Returns a dict (see keys below)."""
    clean = bottom_region(mask)
    L, R, nruns = _trace(clean)
    bands = []
    for i, rows in enumerate(BANDS):
        lo, hi = BAND_EDGES_CM[i], BAND_EDGES_CM[i + 1]
        b = dict(name=f"B{i + 1}", d_lo=lo, d_hi=hi, d_mid=(lo + hi) / 2,
                 y_top=int(rows.min()), y_bot=int(rows.max()),
                 cov=0.0, off_cm=0.0, ratio=0.0, score=0.0, wide=False, split=0.0,
                 L=None, R=None, centre=None, valid=False)
        got = [r for r in rows if L[r] >= 0]
        b['cov'] = len(got) / len(rows)
        if got:
            est = {r: _row_centre(L[r], R[r], r) for r in got}
            info = [r for r in got if est[r][1] != 'none']
            modes = [est[r][1] for r in got]
            main = max(set(modes), key=modes.count)
            if info:
                b['off_cm'] = float(np.median([(est[r][0] - W / 2) * CM_PER_PX[r] for r in info]))
            b['ratio'] = float(np.median([(R[r] - L[r]) / EXP_W_PX[r] for r in got]))
            b['wide'] = b['ratio'] > WIDE_RATIO
            b['split'] = float(np.mean([nruns[r] > 1 for r in got]))
            if b['wide']:
                s_w = 0.25
            elif main == 'both':
                s_w = float(np.exp(-0.5 * ((b['ratio'] - 1) / 0.2) ** 2))
            elif main in ('left', 'right'):
                s_w = 0.8 if b['ratio'] >= 0.3 else 0.4
            else:
                s_w = 0.6
            b['score'] = b['cov'] * s_w * (1 - 0.3 * b['split'])
            b['L'] = float(np.median([L[r] for r in got]))
            b['R'] = float(np.median([R[r] for r in got]))
            b['centre'] = W / 2 + b['off_cm'] / float(np.median([CM_PER_PX[r] for r in got]))
            b['valid'] = True
        bands.append(b)

    good = [b for b in bands if b['valid'] and b['score'] >= MIN_SCORE and b['cov'] >= MIN_COV and not b['wide']]
    if len(good) >= 2:
        slope, icpt = np.polyfit([b['d_mid'] for b in good], [b['off_cm'] for b in good], 1,
                                 w=[b['score'] for b in good])
    elif len(good) == 1:
        slope, icpt = 0.0, good[0]['off_cm']
    else:
        slope, icpt = 0.0, 0.0
    aim = float(icpt + slope * AIM_CM)
    junction = [b['name'] for b in bands[1:] if b['valid'] and b['cov'] > 0.5 and (b['wide'] or b['split'] > 0.5)]
    traced = np.zeros_like(clean)
    for r in range(H):
        if L[r] >= 0:
            traced[r, L[r]:R[r]] = 255
    return dict(bands=bands, clean=traced, lost=len(good) == 0,
                lat_cm=float(icpt), heading=float(np.arctan(slope)), aim_cm=aim,
                steer=float(np.clip(aim / AIM_SCALE_CM, -1, 1)), junction=junction)


def _col(score):
    return (0, 200, 0) if score >= 0.7 else (0, 190, 255) if score >= 0.4 else (0, 0, 255)


def montage(bgr, mask, res, scale=2):
    """original | road as traced for scoring | overlay (raw mask + band scores), scaled up for readable text."""
    big = lambda im: cv2.resize(im, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    clean = res['clean']
    ov = bgr.copy()
    sel = mask > 0
    ov[sel] = (0.55 * ov[sel] + 0.45 * np.array([255, 0, 255])).astype(np.uint8)
    ov = big(ov)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.line(ov, (W * scale // 2, 0), (W * scale // 2, H * scale), (255, 255, 255), 1)   # camera axis
    for b in res['bands']:
        yt, yb = b['y_top'] * scale, (b['y_bot'] + 1) * scale
        cv2.line(ov, (0, yt), (W * scale, yt), (200, 200, 200), 1)
        ym = (yt + yb) // 2
        if b['valid']:
            c = _col(b['score'])
            cv2.line(ov, (int(b['L'] * scale), ym), (int(b['R'] * scale), ym), c, 2)
            cv2.circle(ov, (int(b['centre'] * scale), ym), 4, c, -1)
            tag = f"{b['name']} {b['score']:.2f} {b['off_cm']:+.1f}" + (" W" if b['wide'] else "") + (" S" if b['split'] > 0.5 else "")
        else:
            tag = f"{b['name']} --"
        cv2.putText(ov, tag, (3, yt + 11), font, 0.36, (0, 0, 0), 2)
        cv2.putText(ov, tag, (3, yt + 11), font, 0.36, (255, 255, 255), 1)
    bot = (f"steer {res['steer']:+.2f} aim {res['aim_cm']:+.1f}cm hd {np.degrees(res['heading']):+.0f}d")
    cv2.putText(ov, bot, (3, H * scale - 5), font, 0.4, (0, 0, 0), 3)
    cv2.putText(ov, bot, (3, H * scale - 5), font, 0.4, (0, 255, 255), 1)
    if res['lost']:
        cv2.putText(ov, "ROAD LOST", (110, 24), font, 0.6, (0, 0, 255), 2)
    elif res['junction']:
        cv2.putText(ov, "JUNCTION " + ",".join(res['junction']), (110, 24), font, 0.5, (0, 0, 255), 2)
    return np.hstack([big(bgr), big(cv2.cvtColor(clean, cv2.COLOR_GRAY2BGR)), ov])


def table(res):
    print("  band  dist(cm)   cov  ratio  offset(cm)  score  flags")
    for b in res['bands']:
        fl = ("wide " if b['wide'] else "") + ("split" if b['split'] > 0.5 else "")
        print(f"  {b['name']}   {b['d_lo']:4.1f}-{b['d_hi']:4.1f}   {b['cov']:.2f}  {b['ratio']:5.2f}  {b['off_cm']:+8.2f}   {b['score']:.2f}  {fl}")
    print(f"  fit: lateral {res['lat_cm']:+.2f} cm, heading {np.degrees(res['heading']):+.1f} deg, "
          f"aim {res['aim_cm']:+.2f} cm -> steer {res['steer']:+.2f}"
          + ("   ROAD LOST" if res['lost'] else "") + (f"   junction {res['junction']}" if res['junction'] else ""))


if __name__ == "__main__":
    from road_seg import road_mask
    for f in sys.argv[1:]:
        img = cv2.imread(f)
        if img is None:
            print(f"cannot read {f}")
            continue
        mask = road_mask(img)
        res = analyse(mask)
        out = f.rsplit(".", 1)[0] + "_score.png"
        cv2.imwrite(out, montage(img, mask, res))
        print(f"{f} -> {out}")
        table(res)
