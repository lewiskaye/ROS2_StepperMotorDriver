# Python Imports
import math

# ROS Imports
from sympy import false, true
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

from rclpy.qos import QoSPresetProfiles #QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from custom_interfaces.action import StepperMotor
from custom_interfaces.action import Level

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
STEP_DELAY = 0.005
# Define the number of steps your stepper motor has per revoloution (usual values - 360, 200, 48...)
# Can be calculated as (360 deg / Step deg)
STEPS_PER_REV = 48
# The Microstepping Resoloution (default 1 if not set) (e.g. for 1/8th Resoloution, set to 8)
MICROSETPPING_RES = 8

# Topic to subscribe to IMU Data on (used for angle targeting, self levelling etc.)
IMU_TOPIC = 'imu/imu'

# Default Rotation Directions
CLOCKWISE = 1
ANTI_CLOCKWISE = 0

# Convert a quaternion into euler angles (roll, pitch, yaw)
def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z) # in radians


class StepperService(Node):

    def __init__(self):
        super().__init__('stepper_service')

        # Start the Node and Log
        self.get_logger().info('Initialising Stepper Motor Driver Node...')

        # Create a new instance of the Stepper Driver class
        self.stepperDriver = StepperDriver(STEP_PIN, DIRECTION_PIN, STEP_DELAY, STEPS_PER_REV)

        # Create Action Servers
        self._move_service = ActionServer(self, StepperMotor, 'move_motor', self.move_motor_callback)
        self._level_service = ActionServer(self, Level, 'level_motor', self.level_motor_callback)
        self.levelling = false #stores whether the driver is in a levelling state or not
        #self._level_service = self.create_service(StepperMotor, 'level_motor', self.level_motor_callback)
        # TODO DONE - Shift Away from AddTwoInts and impliment my own custom message type
        # TODO DONE - Change 2nd parameter to Delay

        # Subscribe to IMU topic for Levelling service
        self.imu_subscription = self.create_subscription(Imu, IMU_TOPIC, self.imu_listener_callback, QoSPresetProfiles.SENSOR_DATA.value)
        self.imu_subscription  # prevent unused variable warning

        # Log
        self.get_logger().info('Step Pin: ' + str(STEP_PIN) + ' Direction Pin: ' + str(DIRECTION_PIN) + ' Default Delay: ' + str(STEP_DELAY))
        self.get_logger().info('Steps Per Rev: ' + str(STEPS_PER_REV) + ' Microstepping Res: ' + str(MICROSETPPING_RES))


    # When IMU Data is recieved - log it
    def imu_listener_callback(self, msg):
        self.latest_imu_reading = msg.orientation
        roll, pitch, yaw = euler_from_quaternion(msg.orientation)
        self.latest_pitch = pitch
        #print("Updating: " + str(self.latest_pitch)) #DEBUG
        
        # Check if the driver is meant to be self-levelling currently
        if (self.levelling == true) :
            self.level(pitch, 1) #pitch, tolerance
            sleep(0.01) # Sleep so as to not overwhelm the motor driver!
        

    def level(self, pitch, tolerance=1):
        print("Pitch: " + str(self.latest_pitch)+ " deg")
        
        # Check if within tolerance (1 deg)
        if (-tolerance <= pitch <= tolerance):
            print("Motor is within tolerance of being level.  Marking as SUCCEEDED")
            self.levelling = false
            #self.level_goal_handle.suceed() #issue with this

        elif (pitch > 0):
            print("+ve | moving clockwise")
            self.stepperDriver.Step(1, CLOCKWISE, 0.01)

        elif (pitch < 0):
            print("-ve | moving anti-clockwise")
            self.stepperDriver.Step(1, ANTI_CLOCKWISE, 0.01)


    # When called once - Levels the Motor according to the IMU angle (subscribed)
    def level_motor_callback(self, goal_handle):
        print("Begin Levelling the Motor")
        self.levelling = true

        # Extract Data from the ROS Action Message/Interface & create a response
        self.level_goal_handle = goal_handle
        imu_topic = goal_handle.request.imu_topic #Angle in degrees (relative, not target angle)
        response = Level.Result()

        # Wait for 
        while (self.levelling == false):
            print("waiting for level...")
            sleep(1)

        response.succeeded = int(0)# Return successful '0' Message
        return response

        # print("SUCCESS - Motor is within tolerance of being level")
        # self.levelling = false
        # goal_handle.succeed()
        # response.succeeded = int(0)# Return successful '0' Message
        # return response


    # def oscilate(self):
    #     #level
    #     while true:
    #         stepsToTake = 180 / 360 * (STEPS_PER_REV * MICROSETPPING_RES) //
    #         stepsToTake = round(stepsToTake)
    #         self.stepperDriver.Step(stepsToTake, direction, delay)


    # Moves the Motor the requested angle (relatively, NOT absoloute)
    # Request - Relative Target Rotation Angle (in Degrees)
    #           Speed in Rotations (per Second)
    # Response - relative resulting rotation angle in degrees
    def move_motor_callback(self, goal_handle):
        # Extract Data from the ROS Action Message/Interface
        angle = goal_handle.request.target_angle #Angle in degrees (relative, not target angle)
        speed = goal_handle.request.speed #Speed (or delay?) to move the motor at
        delay = STEP_DELAY #Delay between steps (initialise variable at default rate)
        response = StepperMotor.Result()

        #DEBUG
        # print("Angle: " + str(angle))
        # print("Speed: " + str(speed))

        #TODO Publish Feedback

        # If angle passed in correctly
        if angle != 0:
            # Calculate No. of Steps needed to turn (to the nearest whole step)
            stepsToTake = angle / 360 * (STEPS_PER_REV * MICROSETPPING_RES)
            stepsToTake = round(stepsToTake)
            self.get_logger().info('Recieved request for Angle: ' + str(angle) + " deg")
        else:
            #Bad target angle given
            self.get_logger().error('Bad Input Angle')
            return 0

        # If Speed or Delay passed in correctly
        if speed != 0:
            self.get_logger().info('Recieved request for Speed: ' + str(speed)+ " rps (rotations per second")
            delay = (1 / speed) / (STEPS_PER_REV * MICROSETPPING_RES) # Formula = 1/RPS / stepping-resoloution
            self.get_logger().info('Calculated delay: ' + str(delay))
        else:
            # Use Default Speed
            delay = STEP_DELAY
            self.get_logger().warning('No Speed Set - using default value set upon initialisation: ' + str(delay))


        # Default to Clockwise Rotation
        direction = CLOCKWISE

        # Calculate the resulting angle in degrees again for user info (note Target angle was given, exact steps not always possible due to the step resoloution)
        equivDeg = int(round(stepsToTake / MICROSETPPING_RES / STEPS_PER_REV * 360))
        #response.sum = equivDeg     # Setting this to deg value inneficiently because future changes may require a True/False return value

        # Handle Negative (reverse) values
        if stepsToTake > 0:
            direction = CLOCKWISE
            self.get_logger().info('Stepping Clockwise (' + str(direction) + '): ' + str(stepsToTake) + ' micro-steps (' + str(stepsToTake / MICROSETPPING_RES) + ' full-steps), approx ' + str(equivDeg) + ' deg')
        elif stepsToTake < 0:
            direction = ANTI_CLOCKWISE
            stepsToTake = abs(stepsToTake)  # Make Positive Number of Steps
            self.get_logger().info('Stepping Anti-Clockwise (' + str(direction) + '): ' + str(stepsToTake) + ' micro-steps (' + str(stepsToTake / MICROSETPPING_RES) + ' full-steps), approx ' + str(equivDeg) + ' deg')
        else:
            #ERROR bad target angle after calculation
            self.get_logger().error('Bad Resulting Step Angle.  You may have entered a value less than the step resoloution of your motor')
            #Return Error '1' message
            response.succeeded = int(1)
            goal_handle.fail()
            return response

        # Step the Motor at desired speed
        self.stepperDriver.Step(stepsToTake, direction, delay)

        # Log
        self.get_logger().info('Completed Motor Rotation')

        # Return successful '0' Message
        goal_handle.succeed()
        response.succeeded = int(0)
        return response


def main(args=None):
    rclpy.init(args=args)

    stepper_action_server = StepperService()

    rclpy.spin(stepper_action_server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stepper_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
