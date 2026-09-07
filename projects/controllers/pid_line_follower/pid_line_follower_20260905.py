import webots
from math import radians, sin, cos, sqrt, atan2

class PIDLineFollower:
    def __init__(self):
        self.robot = webots.Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Line sensors
        self.ir_left = self.robot.getDevice('line sensor left')
        self.ir_right = self.robot.getDevice('line sensor right')
        self.ir_center = self.robot.getDevice('line sensor center')
        
        # PID parameters
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.5
        
        # PID state variables
        self.error_sum = 0.0
        self.last_error = 0.0
        self.integral_time = 0.0
        
        # Initialize motors to stop
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
    def raw_state_from_sensors(self, left_black, right_black):
        """Convert sensor readings to a state value"""
        if left_black and right_black:
            return 0.0  # On line
        elif left_black:
            return -1.0  # Left of line
        else:
            return 1.0   # Right of line
            
    def next_state(self, left_black, right_black):
        """Calculate the error using PID control"""
        state = self.raw_state_from_sensors(left_black, right_black)
        
        # Calculate proportional term
        p_term = self.Kp * state
        
        # Calculate integral term
        self.error_sum += state * self.timestep
        i_term = self.Ki * self.error_sum
        
        # Calculate derivative term
        current_error = state
        d_term = self.Kd * (current_error - self.last_error) / self.timestep
        self.last_error = current_error
        
        # Total control output
        control_output = p_term + i_term + d_term
        
        return control_output
        
    def drive_for_state(self, state):
        """Drive the robot based on PID control output"""
        # Convert state to motor velocities
        left_velocity = 1.0 - state * 0.5
        right_velocity = 1.0 + state * 0.5
        
        # Clamp velocities to valid range
        left_velocity = max(-1.0, min(1.0, left_velocity))
        right_velocity = max(-1.0, min(1.0, right_velocity))
        
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)
        
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) == webots.Robot.STEP_OK:
            # Read sensor values (1 = black, 0 = white)
            left_black = self.ir_left.getValue() > 0.5
            right_black = self.ir_right.getValue() > 0.5
            
            # Calculate PID control output
            state = self.next_state(left_black, right_black)
            
            # Drive robot based on control output
            self.drive_for_state(state)
            
        return True


if __name__ == '__main__':
    controller = PIDLineFollower()
    controller.run()
