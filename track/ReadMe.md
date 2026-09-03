Files: peanut_track.png → projects/worlds/textures/peanut_track.png, tpbot.wbt → projects/worlds/tpbot.wbt (updated), generate_peanut_track.py included so you can regenerate it if the measurements change.

The waist bend isn't a free hand-drawn curve — it's fully determined by your three measurements. The connecting arc between each rounded end has to be tangent to that end's circle (no corner, matching the smooth curve in your photo) and pass through the waist width you gave. Those two constraints pin down one specific radius:

Rc = (w² + d² - R1²) / (2×(R1 - w))

where R1 = end radius (148.5mm), d = half the gap between end-circle centres (126.5mm), w = waist half-width (123.5mm). Solving: Rc ≈ 184mm.

Worth flagging directly: that's noticeably different from the ~110mm you measured off the real track with the chord/sagitta method a few turns back. Two possible reasons, and I can't tell which from here:

The chord/sagitta measurement was taken at a slightly different point than the true tightest curvature, or has some hand-measurement error (12mm sagitta over a 100mm chord is a small, error-sensitive measurement).
Real print/fold tolerances mean the physical track doesn't exactly match a single constant-radius arc — real curves drift.

For this PNG, I used the value that's self-consistent with your three stated dimensions (184mm), since that's what you asked me to replicate exactly. But if you're using 110mm anywhere in the turn-radius/speed analysis from earlier, that number was based on the direct physical measurement, not this derived one — worth deciding which one you trust more before relying on either for the speed ceiling calculation.

Also updated in tpbot.wbt: the floor Plane size now matches the PNG's real-world canvas (359mm × 612mm) exactly, and the robot's start position moved onto the track line itself (world origin is empty space between the two lobes).

