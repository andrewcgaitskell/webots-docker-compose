"""
Differential Drive Robot Controller
Controls a two-wheeled robot with IR sensors
"""

from controller import Robot

# Create robot instance
robot = Robot()

# Get simulation timestep (in milliseconds)
timestep = int(robot.getBasicTimeStep())

# Initialize motors
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')

# Set motors to velocity control mode (instead of position control)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Initialize wheel position sensors (encoders)
left_sensor = robot.getDevice('left_sensor')
right_sensor = robot.getDevice('right_sensor')
left_sensor.enable(timestep)
right_sensor.enable(timestep)

# Initialize IR sensors
ir_left = robot.getDevice('ir_left')
ir_right = robot.getDevice('ir_right')
ir_left.enable(timestep)
ir_right.enable(timestep)

# Control parameters
MAX_SPEED = 6.28  # rad/s (approximately 1 rotation per second)

print("Differential Robot Controller Started")
print("=" * 50)

# Main control loop
while robot.step(timestep) != -1:
    # Read sensor values
    left_ir_value = ir_left.getValue()
    right_ir_value = ir_right.getValue()
    left_encoder = left_sensor.getValue()
    right_encoder = right_sensor.getValue()
    
    # Simple forward motion
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED
    
    # Set motor velocities
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Debug output (every second)
    if int(robot.getTime()) % 1 == 0:
        print(f"Time: {robot.getTime():.1f}s | "
              f"IR L: {left_ir_value:.0f} R: {right_ir_value:.0f} | "
              f"Enc L: {left_encoder:.2f} R: {right_encoder:.2f}")


