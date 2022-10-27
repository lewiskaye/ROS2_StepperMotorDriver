# Declare Constants Here

# Define pins - Default GPIO16 for Step, GPIO21 for Direction
STEP_PIN = 16
DIRECTION_PIN = 21
# Define Gap between each step the motor takes in seconds (note - if just testing/starting out, it's probably best to start off with a delay of ~0.1)
STEP_DELAY = 0.005
# Define the number of steps your stepper motor has per revoloution (usual values - 360, 200, 48...)
# Can be calculated as (360 deg / Step deg)
STEPS_PER_REV = 48
# The Microstepping Resoloution (default 1 if not set) (e.g. for 1/8th Resoloution, set to 8)
MICROSETPPING_RES = 8

# Default Rotation Directions
CLOCKWISE = 1
ANTI_CLOCKWISE = 0