# System imports
import RPi.GPIO as GPIO
from time import sleep

class StepperDriver():

	__CLOCKWISE = 1
	__ANTI_CLOCKWISE = 0

	def __init__(self, stepPin, directionPin, delay=0.208, stepsPerRevolution=48):

		# Configure instance
		self.CLOCKWISE = self.__CLOCKWISE
		self.ANTI_CLOCKWISE = self.__ANTI_CLOCKWISE
		self.StepPin = stepPin
		self.DirectionPin = directionPin
		self.Delay = delay
		self.RevolutionSteps = stepsPerRevolution
		self.CurrentDirection = self.CLOCKWISE
		self.CurrentStep = 0

		# Setup gpio pins
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.StepPin, GPIO.OUT)
		GPIO.setup(self.DirectionPin, GPIO.OUT)

	def Step(self, stepsToTake, direction = __CLOCKWISE, delay=0):
		
		# Set Delay if present
		if delay != 0:
			self.Delay = delay
		
		# Interperet stepsToTake as Integer
		stepsToTake = int(stepsToTake)
		print("Taking " + str(stepsToTake) + " steps.")

		# Set the direction
		GPIO.output(self.DirectionPin, direction)

		# Take requested number of steps
		for x in range(stepsToTake):
			#print("Step " + str(x)) #DEBUG
			GPIO.output(self.StepPin, GPIO.HIGH)
			self.CurrentStep += 1
			sleep(self.Delay)
			GPIO.output(self.StepPin, GPIO.LOW)
			sleep(self.Delay)
		
		# Finished
		print("Finished Stepping Motor")
