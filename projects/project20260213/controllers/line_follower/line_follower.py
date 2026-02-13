"""
Line Following Controller
Uses two IR sensors to follow a dark line on light surface
"""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# IR Sensors
ir_left = robot.getDevice('ir_left')
ir_right = robot.getDevice('ir_right')
ir_left.enable(timestep)
ir_right.enable(timestep)

# Parameters
MAX_SPEED = 5.0
THRESHOLD = 500  # IR threshold for line detection (adjust based on your surface)

print("Line Following Controller Started")

while robot.step(timestep) != -1:
    # Read IR sensors
    left_value = ir_left.getValue()
    right_value = ir_right.getValue()
    
    # Detect line (high value = dark surface/line)
    left_on_line = left_value > THRESHOLD
    right_on_line = right_value > THRESHOLD
    
    # Line following logic
    if left_on_line and right_on_line:
        # Both sensors on line → Go straight
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
        state = "STRAIGHT"
        
    elif left_on_line and not right_on_line:
        # Only left sensor on line → Turn left
        left_speed = MAX_SPEED * 0.2
        right_speed = MAX_SPEED
        state = "TURN LEFT"
        
    elif not left_on_line and right_on_line:
        # Only right sensor on line → Turn right
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED * 0.2
        state = "TURN RIGHT"
        
    else:
        # No sensors on line → Search/rotate
        left_speed = MAX_SPEED * 0.5
        right_speed = -MAX_SPEED * 0.5
        state = "SEARCHING"
    
    # Apply speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Debug every 0.5 seconds
    current_time = robot.getTime()
    if int(current_time * 2) % 1 == 0 and current_time * 2 - int(current_time * 2) < 0.1:
        print(f"[{current_time:6.2f}s] State: {state:12s} | "
              f"IR L:{left_value:5.0f} R:{right_value:5.0f}")

  
