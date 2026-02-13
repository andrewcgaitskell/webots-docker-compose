"""
PID-based Line Following Controller
Smoother line following using proportional-derivative control
"""

from controller import Robot

class PIDLineFollower:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Motors
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # IR Sensors
        self.ir_left = self.robot.getDevice('ir_left')
        self.ir_right = self.robot.getDevice('ir_right')
        self.ir_left.enable(self.timestep)
        self.ir_right.enable(self.timestep)
        
        # PID parameters
        self.kp = 0.005  # Proportional gain
        self.kd = 0.001  # Derivative gain
        self.base_speed = 4.0
        self.max_speed = 6.28
        
        self.last_error = 0
        
    def run(self):
        print("PID Line Follower Started")
        
        while self.robot.step(self.timestep) != -1:
            # Read sensors
            left = self.ir_left.getValue()
            right = self.ir_right.getValue()
            
            # Calculate error (positive = line to the right)
            error = right - left
            
            # PID control
            derivative = error - self.last_error
            correction = self.kp * error + self.kd * derivative
            
            # Calculate wheel speeds
            left_speed = self.base_speed + correction
            right_speed = self.base_speed - correction
            
            # Clamp speeds
            left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
            right_speed = max(-self.max_speed, min(self.max_speed, right_speed))
            
            # Apply speeds
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
            
            # Update last error
            self.last_error = error
            
            # Debug
            if int(self.robot.getTime() * 5) % 5 == 0:
                print(f"[{self.robot.getTime():5.1f}s] "
                      f"Error:{error:6.0f} Corr:{correction:6.3f} | "
                      f"L_spd:{left_speed:5.2f} R_spd:{right_speed:5.2f}")

if __name__ == "__main__":
    controller = PIDLineFollower()
    controller.run()



