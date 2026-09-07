"""
PID line following controller for TPBot, refactored from the bang-bang
version (which was itself a port of the real C++/CODAL firmware state
machine in main.cpp).

https://github.com/andrewcgaitskell/tpbot-cplusplus-codal

Difference from bang-bang: instead of thresholding each IR sensor into a
binary on/off reading and switching between discrete states (Straight,
CorrectingLeft/Right, SearchingLeft/Right), this version treats the two
raw analog sensor readings as a continuous error signal and drives a
standard PID loop to correct heading. The line is only truly lost when
BOTH sensors drop below a "line present" threshold - in that case we fall
back on the same last-known-direction search behaviour as the bang-bang
version, since a 2-sensor rig genuinely has nothing better to go on.

Speed calibration is unchanged: the firmware's setWheels() takes
arbitrary 0-100-ish "speed units". Real robot measurements gave 136mm/s
at speed unit 15, so that's used here as the scale factor to convert
every other tuned speed constant into real mm/s, then into wheel angular
velocity (rad/s) via the 30mm wheel radius from TPBot.proto.
"""

from controller import Robot

# --- Physical constants (must match TPBot.proto) ---
WHEEL_RADIUS_M = 0.03  # Cylinder radius in TPBot.proto

# --- Speed calibration ---
# Measured on real hardware: BASE_SPEED=15 (firmware units) -> 136mm/s over
# a 1.5m straight (11s run). Every other speed constant below is converted
# through this same scale factor, so ratios match the real robot, not just
# absolute numbers.
MM_PER_SPEED_UNIT = 136.0 / 15.0

# --- Firmware speed constants (firmware units, copied from main.cpp) ---
BASE_SPEED_UNITS = 15
TRIM_UNITS = 3
SEARCH_OUTER_UNITS = 40
SEARCH_INNER_UNITS = -10

# --- PID gains (firmware "speed units" of correction per unit of
# normalised error, i.e. tuned as if error were in the range -1..1) ---
# Start conservative and tune on the real robot / in Webots: increase KP
# until you get a fast but not oscillating response, then add KD to damp
# overshoot, then a small KI only if there's a persistent steady-state
# bias (e.g. from motor mismatch).
KP_UNITS = 30.0
KI_UNITS = 0.0
KD_UNITS = 8.0

# Clamp on the PID correction, in firmware speed units, so a bad reading
# (e.g. momentarily losing both sensors right at threshold) can't slam the
# motors to full opposite speed.
MAX_CORRECTION_UNITS = 35.0

# Clamp on the integral term's contribution (anti-windup), same units as
# MAX_CORRECTION_UNITS.
MAX_INTEGRAL_UNITS = 20.0

# --- Timing (must match firmware's tuned values) ---
LOST_LINE_TIMEOUT_MS = 600

# --- Sensor thresholds ---
# TPBot.proto's lookupTable returns ~0 over white, ~1000 over black (see
# the comment above the DistanceSensor nodes in the PROTO for the
# derivation).
# BLACK_THRESHOLD is kept only for reference/tuning notes; the PID loop
# uses the raw analog values directly rather than thresholding them.
BLACK_THRESHOLD = 500
# A sensor reading below this is treated as "seeing nothing" (pure white)
# for the purposes of deciding whether the line is lost. Keep this well
# below BLACK_THRESHOLD so partial/edge-of-line readings still count as
# "line present" and feed the PID loop rather than triggering search mode.
LINE_PRESENT_THRESHOLD = 100


def units_to_rad_s(units):
    """Convert a firmware speed unit (or delta of units) into wheel
    angular velocity (rad/s)."""
    mm_per_s = units * MM_PER_SPEED_UNIT
    m_per_s = mm_per_s / 1000.0
    return m_per_s / WHEEL_RADIUS_M


# Pre-converted, real-world-calibrated wheel speeds.
BASE_RAD = units_to_rad_s(BASE_SPEED_UNITS)
TRIM_RAD = units_to_rad_s(TRIM_UNITS)
SEARCH_OUTER_RAD = units_to_rad_s(SEARCH_OUTER_UNITS)
SEARCH_INNER_RAD = units_to_rad_s(SEARCH_INNER_UNITS)
MAX_CORRECTION_RAD = units_to_rad_s(MAX_CORRECTION_UNITS)

# States - SEARCHING/STOPPED names match FollowState in main.cpp for easy
# comparison; PID_TRACKING is new (replaces Straight/CorrectingLeft/Right).
PID_TRACKING = "PidTracking"
SEARCHING_LEFT = "SearchingLeft"
SEARCHING_RIGHT = "SearchingRight"
STOPPED = "Stopped"

DIRECTION_UNKNOWN = 0
DIRECTION_LEFT = 1
DIRECTION_RIGHT = -1


class PIDController:
    """Plain PID over a normalised error in roughly [-1, 1]. Gains and
    output are expressed in firmware speed units so they're directly
    comparable to the bang-bang controller's tuned constants."""

    def __init__(self, kp, ki, kd, output_limit, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_error_valid = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_error_valid = False

    def update(self, error, dt_s):
        self.integral += error * dt_s
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        if self.prev_error_valid and dt_s > 0:
            derivative = (error - self.prev_error) / dt_s
        else:
            derivative = 0.0

        self.prev_error = error
        self.prev_error_valid = True

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(-self.output_limit, min(self.output_limit, output))


class PIDLineFollower:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.dt_s = self.timestep / 1000.0

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.ir_left = self.robot.getDevice("line sensor left")
        self.ir_right = self.robot.getDevice("line sensor right")
        self.ir_left.enable(self.timestep)
        self.ir_right.enable(self.timestep)

        self.pid = PIDController(
            kp=units_to_rad_s(KP_UNITS),
            ki=units_to_rad_s(KI_UNITS),
            kd=units_to_rad_s(KD_UNITS),
            output_limit=MAX_CORRECTION_RAD,
            integral_limit=units_to_rad_s(MAX_INTEGRAL_UNITS),
        )

        self.last_known_direction = DIRECTION_UNKNOWN
        self.ms_since_line_seen = 0
        self.state = STOPPED

    @staticmethod
    def compute_error(left_val, right_val):
        """Normalised error in [-1, 1]. Positive means more line under the
        left sensor than the right, i.e. the robot needs to turn left to
        recentre (mirrors the bang-bang CorrectingLeft convention: left
        wheel slower, right wheel faster)."""
        total = left_val + right_val
        if total <= 0:
            return 0.0
        return (left_val - right_val) / total

    def next_state(self, left_val, right_val):
        line_present = (left_val >= LINE_PRESENT_THRESHOLD) or (right_val >= LINE_PRESENT_THRESHOLD)

        if line_present:
            self.ms_since_line_seen = 0
            # Track which side is currently dominant so a brief future
            # drop-out has a direction to search toward.
            if left_val > right_val:
                self.last_known_direction = DIRECTION_LEFT
            elif right_val > left_val:
                self.last_known_direction = DIRECTION_RIGHT
            self.state = PID_TRACKING
            return self.state

        # Neither sensor sees the line.
        self.ms_since_line_seen += self.timestep
        self.pid.reset()  # avoid integral/derivative kick when line reappears

        if self.ms_since_line_seen > LOST_LINE_TIMEOUT_MS:
            self.state = STOPPED
        elif self.last_known_direction == DIRECTION_LEFT:
            self.state = SEARCHING_LEFT
        elif self.last_known_direction == DIRECTION_RIGHT:
            self.state = SEARCHING_RIGHT
        else:
            self.state = STOPPED  # never had a fix - nothing to search toward

        return self.state

    def drive(self, state, left_val, right_val):
        if state == PID_TRACKING:
            error = self.compute_error(left_val, right_val)
            correction = self.pid.update(error, self.dt_s)
            self.left_motor.setVelocity(BASE_RAD + TRIM_RAD - correction)
            self.right_motor.setVelocity(BASE_RAD - TRIM_RAD + correction)
        elif state == SEARCHING_LEFT:
            self.left_motor.setVelocity(SEARCH_INNER_RAD)
            self.right_motor.setVelocity(SEARCH_OUTER_RAD)
        elif state == SEARCHING_RIGHT:
            self.left_motor.setVelocity(SEARCH_OUTER_RAD)
            self.right_motor.setVelocity(SEARCH_INNER_RAD)
        else:  # STOPPED
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)

    def run(self):
        print(f"PID line follower started. timestep={self.timestep}ms "
              f"BASE={BASE_RAD:.2f}rad/s "
              f"PID(kp,ki,kd)=({KP_UNITS},{KI_UNITS},{KD_UNITS}) units "
              f"MAX_CORRECTION={MAX_CORRECTION_RAD:.2f}rad/s")

        while self.robot.step(self.timestep) != -1:
            left_val = self.ir_left.getValue()
            right_val = self.ir_right.getValue()

            state = self.next_state(left_val, right_val)
            self.drive(state, left_val, right_val)


if __name__ == "__main__":
    controller = PIDLineFollower()
    controller.run()
    