"""
Bang-bang line following controller for TPBot, ported from the real
C++/CODAL firmware (main.cpp) state machine. No PID.

https://github.com/andrewcgaitskell/tpbot-cplusplus-codal

Two binary sensors (thresholded), same states as the firmware:
  Straight          - both sensors on line
  CorrectingLeft/Right - one sensor on line - gentle nudge, not a spin
  SearchingLeft/Right  - line lost, keep turning the last known direction
  Stopped           - line lost too long, or never had a fix

Speed calibration: the firmware's setWheels() takes arbitrary 0-100-ish
"speed units". Real robot measurements gave 136mm/s at speed unit 15, so
that's used here as the scale factor to convert every other tuned speed
constant into real mm/s, then into wheel angular velocity (rad/s) via the
30mm wheel radius from TPBot.proto. This means a speed sweep in sim can be
compared directly against real 1.5m-straight timing runs.
"""

from controller import Robot

# --- Physical constants (must match TPBot.proto) ---
WHEEL_RADIUS_M = 0.03  # Cylinder radius in TPBot.proto

# --- Speed calibration ---
# Measured on real hardware: BASE_SPEED=15 (firmware units) -> 136mm/s over
# a 1.5m straight (11s run). Every other firmware speed constant below is
# converted through this same scale factor, so the ratios between states
# (correction vs base vs search) match the real robot, not just the
# absolute numbers.
MM_PER_SPEED_UNIT = 136.0 / 15.0

# --- Firmware speed constants (firmware units, copied from main.cpp) ---
BASE_SPEED_UNITS = 15
TRIM_UNITS = 3
CORRECT_OUTER_UNITS = 22
CORRECT_INNER_UNITS = 6
SEARCH_OUTER_UNITS = 40
SEARCH_INNER_UNITS = -10

# --- Timing (must match firmware's tuned values) ---
LOST_LINE_TIMEOUT_MS = 600
DEBOUNCE_READS = 2

# --- Sensor threshold ---
# TPBot.proto's lookupTable returns ~0 over white, ~1000 over black (see the
# comment above the DistanceSensor nodes in the PROTO for the derivation).
# Midpoint threshold - if this doesn't cleanly separate black/white in
# practice, print raw getValue() readings over both colors and re-centre
# this rather than guessing.
BLACK_THRESHOLD = 500


def units_to_rad_s(units):
    """Convert a firmware speed unit into wheel angular velocity (rad/s)."""
    mm_per_s = units * MM_PER_SPEED_UNIT
    m_per_s = mm_per_s / 1000.0
    return m_per_s / WHEEL_RADIUS_M


# Pre-converted, real-world-calibrated wheel speeds.
BASE_RAD = units_to_rad_s(BASE_SPEED_UNITS)
TRIM_RAD = units_to_rad_s(TRIM_UNITS)
CORRECT_OUTER_RAD = units_to_rad_s(CORRECT_OUTER_UNITS)
CORRECT_INNER_RAD = units_to_rad_s(CORRECT_INNER_UNITS)
SEARCH_OUTER_RAD = units_to_rad_s(SEARCH_OUTER_UNITS)
SEARCH_INNER_RAD = units_to_rad_s(SEARCH_INNER_UNITS)

# States - names match FollowState in main.cpp exactly, for easy comparison.
STRAIGHT = "Straight"
CORRECTING_LEFT = "CorrectingLeft"
CORRECTING_RIGHT = "CorrectingRight"
SEARCHING_LEFT = "SearchingLeft"
SEARCHING_RIGHT = "SearchingRight"
STOPPED = "Stopped"

DIRECTION_UNKNOWN = 0
DIRECTION_LEFT = 1
DIRECTION_RIGHT = -1


class BangBangLineFollower:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.ir_left = self.robot.getDevice("line sensor left")
        self.ir_right = self.robot.getDevice("line sensor right")
        self.ir_left.enable(self.timestep)
        self.ir_right.enable(self.timestep)

        self.last_known_direction = DIRECTION_UNKNOWN
        self.pending_state = STOPPED
        self.accepted_state = STOPPED
        self.match_count = 0
        self.ms_since_line_seen = 0

    def raw_state_from_sensors(self, left_black, right_black):
        if left_black and right_black:
            self.last_known_direction = DIRECTION_UNKNOWN
            return STRAIGHT

        if left_black and not right_black:
            self.last_known_direction = DIRECTION_LEFT
            return CORRECTING_LEFT

        if not left_black and right_black:
            self.last_known_direction = DIRECTION_RIGHT
            return CORRECTING_RIGHT

        # Neither sensor sees the line - fall back on history.
        if self.ms_since_line_seen > LOST_LINE_TIMEOUT_MS:
            return STOPPED

        if self.last_known_direction == DIRECTION_LEFT:
            return SEARCHING_LEFT
        elif self.last_known_direction == DIRECTION_RIGHT:
            return SEARCHING_RIGHT
        else:
            return STOPPED  # never had a fix - nothing to search toward

    def next_state(self, left_black, right_black):
        raw = self.raw_state_from_sensors(left_black, right_black)

        # Searching/Stopped are already gated on ms_since_line_seen, not a
        # single noisy read - apply immediately, same as the firmware.
        if raw in (SEARCHING_LEFT, SEARCHING_RIGHT, STOPPED):
            self.accepted_state = raw
            self.pending_state = raw
            self.match_count = 0
            return self.accepted_state

        if raw == self.pending_state:
            self.match_count += 1
        else:
            self.pending_state = raw
            self.match_count = 1

        if self.match_count >= DEBOUNCE_READS:
            self.accepted_state = self.pending_state

        return self.accepted_state

    def drive_for_state(self, state):
        if state == STRAIGHT:
            self.left_motor.setVelocity(BASE_RAD + TRIM_RAD)
            self.right_motor.setVelocity(BASE_RAD - TRIM_RAD)
        elif state == CORRECTING_LEFT:
            self.left_motor.setVelocity(CORRECT_INNER_RAD)
            self.right_motor.setVelocity(CORRECT_OUTER_RAD)
        elif state == CORRECTING_RIGHT:
            self.left_motor.setVelocity(CORRECT_OUTER_RAD)
            self.right_motor.setVelocity(CORRECT_INNER_RAD)
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
        print(f"Bang-bang line follower started. timestep={self.timestep}ms "
              f"BASE={BASE_RAD:.2f}rad/s CORRECT=({CORRECT_INNER_RAD:.2f},{CORRECT_OUTER_RAD:.2f}) "
              f"SEARCH=({SEARCH_INNER_RAD:.2f},{SEARCH_OUTER_RAD:.2f})")

        while self.robot.step(self.timestep) != -1:
            left_black = self.ir_left.getValue() > BLACK_THRESHOLD
            right_black = self.ir_right.getValue() > BLACK_THRESHOLD

            if left_black or right_black:
                self.ms_since_line_seen = 0
            else:
                self.ms_since_line_seen += self.timestep

            state = self.next_state(left_black, right_black)
            self.drive_for_state(state)


if __name__ == "__main__":
    controller = BangBangLineFollower()
    controller.run()
