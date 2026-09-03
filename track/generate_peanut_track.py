"""
Generates a peanut/dogbone-shaped track PNG matching real-world measurements:
  - overall length (long axis):     550mm
  - rounded end width (diameter):   297mm
  - waist width (narrowest point):  247mm
  - line (stroke) width:            22mm

Geometry: two circles (the rounded ends) connected by two symmetric
"reverse curve" arcs (the waist bends) that are tangent-continuous with
both circles - i.e. no corner at the waist, matching the smooth curve
visible in the real track photo. The waist arc radius isn't a free
choice; it's fully determined by the three measurements above (solved
algebraically below), so the three real-world dimensions are all
satisfied exactly, not just approximately.
"""

import math
import cairosvg

# --- Real-world track dimensions (mm) ---
LENGTH_MM = 550.0
END_WIDTH_MM = 297.0
WAIST_WIDTH_MM = 247.0
LINE_WIDTH_MM = 22.0

# --- Rendering ---
PX_PER_MM = 4.0
MARGIN_MM = 20.0

R1 = END_WIDTH_MM / 2.0          # end circle radius
d = (LENGTH_MM - END_WIDTH_MM) / 2.0  # half the gap between the two circle centres
w = WAIST_WIDTH_MM / 2.0         # half-width at the waist

# Solve for the waist connecting-arc radius Rc such that the arc is
# externally tangent to the end circle (reverse-curve / S-tangent, since
# the waist curves the opposite way to the end caps) AND passes exactly
# through the measured waist half-width w on the centre axis.
#   (w + Rc)^2 + d^2 = (R1 + Rc)^2   =>   Rc = (w^2 + d^2 - R1^2) / (2*(R1 - w))
Rc = (w**2 + d**2 - R1**2) / (2.0 * (R1 - w))
if Rc <= 0:
    raise ValueError(f"Computed Rc={Rc:.2f} is not positive - check input dimensions")

ox = w + Rc  # x-coordinate of the right-side waist arc centre (left side is mirrored, -ox)

# Circle centres (track centred on origin, y-axis = long axis)
C1 = (0.0, d)    # top end circle
C2 = (0.0, -d)   # bottom end circle
O_right = (ox, 0.0)
O_left = (-ox, 0.0)


def angle_deg(center, point):
    return math.degrees(math.atan2(point[1] - center[1], point[0] - center[0]))


def point_on_circle(center, radius, angle_deg_):
    a = math.radians(angle_deg_)
    return (center[0] + radius * math.cos(a), center[1] + radius * math.sin(a))


# Tangent point between the right waist arc and the top circle (C1):
# for external tangency, it lies on the line joining the two centres,
# at distance R1 from C1.
dx, dy = O_right[0] - C1[0], O_right[1] - C1[1]
dist = math.hypot(dx, dy)
T1 = (C1[0] + R1 * dx / dist, C1[1] + R1 * dy / dist)          # top-right tangent point
T2 = (T1[0], -T1[1])                                            # bottom-right (mirror about y=0)
T1_left = (-T1[0], T1[1])                                       # top-left
T2_left = (-T1[0], -T1[1])                                      # bottom-left

print(f"R1(end radius)={R1:.2f}mm  d(half-gap)={d:.2f}mm  w(waist half-width)={w:.2f}mm")
print(f"Rc(waist arc radius)={Rc:.2f}mm  ox(waist arc centre x)={ox:.2f}mm")
print(f"T1={T1[0]:.2f},{T1[1]:.2f}  T2={T2[0]:.2f},{T2[1]:.2f}")


def arc_points(center, radius, a_start, a_end, n=60):
    """Sample points on a circular arc from a_start to a_end (degrees),
    sweeping in whichever direction covers |a_end-a_start| as given
    (caller is responsible for picking the short-way or long-way delta)."""
    pts = []
    for i in range(n + 1):
        t = a_start + (a_end - a_start) * (i / n)
        pts.append(point_on_circle(center, radius, t))
    return pts


def shortest_delta(a_from, a_to):
    """Signed angle delta in (-180, 180] taking the short way round."""
    delta = (a_to - a_from + 180) % 360 - 180
    return delta


def longest_delta(a_from, a_to):
    """Signed angle delta taking the long way round (magnitude > 180)."""
    short = shortest_delta(a_from, a_to)
    if short >= 0:
        return short - 360
    else:
        return short + 360


# --- Build the closed outline, traversal order: ---
# T1_left --(left waist, short way)--> T2_left
# T2_left --(bottom cap, long way, under the bottom)--> T2
# T2      --(right waist, short way)--> T1
# T1      --(top cap, long way, over the top)--> T1_left  (closes the loop)

a_T1_left_leftwaist = angle_deg(O_left, T1_left)
a_T2_left_leftwaist = angle_deg(O_left, T2_left)
left_waist_pts = arc_points(O_left, Rc, a_T1_left_leftwaist,
                             a_T1_left_leftwaist + shortest_delta(a_T1_left_leftwaist, a_T2_left_leftwaist))

a_T2_left_bottom = angle_deg(C2, T2_left)
a_T2_bottom = angle_deg(C2, T2)
bottom_cap_pts = arc_points(C2, R1, a_T2_left_bottom,
                             a_T2_left_bottom + longest_delta(a_T2_left_bottom, a_T2_bottom))

a_T2_rightwaist = angle_deg(O_right, T2)
a_T1_rightwaist = angle_deg(O_right, T1)
right_waist_pts = arc_points(O_right, Rc, a_T2_rightwaist,
                              a_T2_rightwaist + shortest_delta(a_T2_rightwaist, a_T1_rightwaist))

a_T1_top = angle_deg(C1, T1)
a_T1_left_top = angle_deg(C1, T1_left)
top_cap_pts = arc_points(C1, R1, a_T1_top,
                          a_T1_top + longest_delta(a_T1_top, a_T1_left_top))

outline = left_waist_pts + bottom_cap_pts + right_waist_pts + top_cap_pts

# --- Render to SVG, centred in the canvas ---
bbox_half_w = R1 + LINE_WIDTH_MM / 2.0
bbox_half_h = (d + R1) + LINE_WIDTH_MM / 2.0
canvas_w_mm = 2 * bbox_half_w + 2 * MARGIN_MM
canvas_h_mm = 2 * bbox_half_h + 2 * MARGIN_MM
cx = canvas_w_mm / 2.0
cy = canvas_h_mm / 2.0

path_d = "M " + " L ".join(f"{cx + p[0]:.3f},{cy - p[1]:.3f}" for p in outline) + " Z"

svg = f'''<svg xmlns="http://www.w3.org/2000/svg"
     width="{canvas_w_mm * PX_PER_MM:.0f}" height="{canvas_h_mm * PX_PER_MM:.0f}"
     viewBox="0 0 {canvas_w_mm:.3f} {canvas_h_mm:.3f}">
  <rect x="0" y="0" width="{canvas_w_mm:.3f}" height="{canvas_h_mm:.3f}" fill="#ffffff"/>
  <path d="{path_d}" fill="none" stroke="#000000"
        stroke-width="{LINE_WIDTH_MM:.3f}" stroke-linejoin="round" stroke-linecap="round"/>
</svg>'''

with open("/home/claude/peanut_track.svg", "w") as f:
    f.write(svg)

cairosvg.svg2png(url="/home/claude/peanut_track.svg", write_to="/home/claude/peanut_track.png",
                  output_width=int(canvas_w_mm * PX_PER_MM), output_height=int(canvas_h_mm * PX_PER_MM))

print(f"Canvas: {canvas_w_mm:.1f}mm x {canvas_h_mm:.1f}mm ({canvas_w_mm*PX_PER_MM:.0f}x{canvas_h_mm*PX_PER_MM:.0f}px @ {PX_PER_MM}px/mm)")
print("Wrote peanut_track.svg and peanut_track.png")
