# Python Imports

# ROS Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
#from std_srvs.srv import <custom srv>

# RPi Imports
import RPi.GPIO as GPIO
from time import sleep

# Project Imports
from .StepperDriver import StepperDriver
#import config

# Define pins - Default GPIO16 for Step, GPIO21 for Direction
STEP_PIN = 16
DIRECTION_PIN = 21
# Define Gap between each step the motor takes in seconds (note - if just testing/starting out, it's probably best to start off with a delay of ~0.1)
STEP_DELAY = 0.0025
# Define the number of steps your stepper motor has per revoloution (usual values - 360, 200, 48...)
# Can be calculated as (360 deg / Step deg)
STEPS_PER_REV = 48
# The Microstepping Resoloution (default 1 if not set) (e.g. for 1/8th Resoloution, set to 8)
MICROSETPPING_RES = 16

# Default Rotation Directions
CLOCKWISE = 1
ANTI_CLOCKWISE = 0

class StepperService(Node):

    def __init__(self):
        super().__init__('stepper_service')

        # Start the Node and Log
        self.get_logger().info('Initialising Stepper Motor Driver Node...')

        # Create a new instance of the Stepper Driver class
        self.stepperDriver = StepperDriver(STEP_PIN, DIRECTION_PIN, STEP_DELAY, STEPS_PER_REV)

        # Create Service
        self.srv = self.create_service(AddTwoInts, 'move_motor', self.move_motor_callback)
        # TODO - Shift Away from AddTwoInts and impliment my own custom message type


    # Moves the Motor the requested angle (relatively, NOT absoloute)
    # Request - relative target rotation angle in degrees
    # Response - relative resulting rotation angle in degrees
    def move_motor_callback(self, request, response):
        # Log
        self.get_logger().info('Step Pin: ' + str(STEP_PIN) + ' Direction Pin: ' + str(DIRECTION_PIN) + ' Delay: ' + str(STEP_DELAY))

        # Extract Data from the ROS SRV Message/Interface
        # TODO - Change to custom message instead of ROS2 Example SRV Message
        # If Step Count is given directly (i.e. not just a target angle in degrees)
        if request.b != 0:
            stepsToTake = request.b
            self.get_logger().info('Recieved request for STEP COUNT: ' + str(stepsToTake))

        # Else if steps are given in degrees
        elif request.a != 0:
            # Calculate No. of Steps needed to turn (to the nearest whole step)
            stepsToTake = request.a / 360 * STEPS_PER_REV * MICROSETPPING_RES
            stepsToTake = round(stepsToTake)
            self.get_logger().info('Recieved request for ANGLE: ' + str(stepsToTake))

        else:
            #ERROR bad target angle given
            self.get_logger().error('Bad Input Angle')
            return 0


        # Default to Clockwise Rotation
        direction = CLOCKWISE

        # Calculate the resulting angle in degrees again for user info (note Target angle was given, exact steps not always possible due to the step resoloution)
        equivDeg = int(round(stepsToTake / MICROSETPPING_RES / STEPS_PER_REV * 360))
        #response.sum = equivDeg     # Setting this to deg value inneficiently because future changes may require a True/False return value


        # Handle Negative (reverse) values
        if stepsToTake > 0:
            direction = CLOCKWISE
            self.get_logger().info('Stepping Clockwise: ' + str(stepsToTake) + ' micro-steps (' + str(stepsToTake / MICROSETPPING_RES) + ' full-steps), approx ' + str(equivDeg) + ' deg')
        elif stepsToTake < 0:
            direction = ANTI_CLOCKWISE
            stepsToTake = abs(stepsToTake)  # Make Positive Number of Steps
            self.get_logger().info('Stepping Anti-Clockwise: ' + str(stepsToTake) + ' micro-steps (' + str(stepsToTake / MICROSETPPING_RES) + ' full-steps), approx ' + str(equivDeg) + ' deg')
        else:
            #ERROR bad target angle after calculation
            self.get_logger().error('Bad Resulting Step Angle.  You may have entered a value less than the step resoloution of your motor')
            #Return Error '1' message
            response.sum = 1
            return response


        # Step the Motor
        self.stepperDriver.Step(stepsToTake, direction)


        # Log
        self.get_logger().info('Completed Motor Rotation')
        #self.get_logger().info('Completed Motor Rotation: %d b: %d' % (request.a, request.b))

        # Return successful '0' Message
        response.sum = 0
        return response



def main(args=None):
    rclpy.init(args=args)

    stepper_service = StepperService()

    rclpy.spin(stepper_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stepper_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
