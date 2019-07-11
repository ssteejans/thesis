# -*- coding: utf-8 -*-
#
## @file ActuatorDriver.py
#  This file contains a driver for a linear actuator for use in a 3 DOF parallel 
#  actuator telescope mount.  
#
#
#  @author Samuel Steejans Artho-Bentz

## @class Actuator
#  Object which represents one linear actuator for a 3 DOF parallel actuator telescope mount
#  @param stepPerLength Total resolution of the actuator: STEPPER_RESOLUTION*SCREW_RESOLUTION*MICROSTEPS
#  @param motorController Object which controls the motor: l6470nucleo.Dual6470
#  @param motorNumber Motor selection for the l6470nucleo.Dual6470: 1 or 2
class Actuator(object):

    ## Initialize the Actuator object. 

    def __init__(self, stepPerLength, motorController, motorNumber):
        ## @var stepPerLength
        #  The number of steps the stepper motor must take in order to move
        #  a distance.
        #  Defines the units of length throughout the system.
        self.stepPerLength = stepPerLength
        ## @var controller
        #  The motor controller associated with the actuator.
        self.controller = motorController
        ## @var motorNumber
        #  The motor to be used from the controller
        self.motorNumber = motorNumber
        ## @var currentLength
        #  Variable to track the length of the actuator
        self.currentLength = 0

    ## @fn commandVelocity
    #  Commands the actuator to move at a specified velocity.
    #  Length unit is defined by the stepPerLength parameter.
    #  @param velocity [length/sec]
    def commandVelocity(self, velocity):

        stepsPerSec = self.__calcSteps(velocity)
        print('Command to ' + str(stepsPerSec) + ' steps/sec')
        self.controller.run(self.motorNumber, stepsPerSec)
        
    ## @fn commandLength
    #  Commands the actuator to move to a specific length.
    #  Length unit is defined by the stepPerLength parameter.
    #  @param length [length]
    def commandLength(self, length):
        steps = self.__calcSteps(length)
        print(['command to ' + str(steps) + ' steps'])
        self.__commandStepMotion(steps)        
        
    ## convert length into steps
    def __calcSteps(self, length):
        steps = round(length*self.stepPerLength)
        return steps
        
    ## interface with motorController to move to a specific step count
    def __commandStepMotion(self,steps):
        self.controller.GoTo(self.motorNumber, steps)
    
    ## @fn getCurrentLength
    #  Reads the stepcount of the motor and converts to length
    #  @return currentLength the current length of the actuator
    def getCurrentLength(self):
        self.currentLength = self.controller.getPositions(self.motorNumber)/self.stepPerLength
        return self.currentLength
        
    ## @fn findHome
    #  commands the actuator to slowly shorten.
    #  @b WARNING: no automatic stop is built into this function
    def findHome(self):
        self.controller.run(self.motorNumber, -10)
        
    ## @fn isHome
    #  Checks if the motor is stalled.
    #  @b WARNING: results were not consistent and where very dependent on motor tuning.
    def isHome(self):
        stalled = self.controller.isStalled(self.motorNumber, True)
        if stalled:
            self.controller.HardHiZ(self.motorNumber)
            return True
        else:
            return False
            
    ## @fn resetPosition
    #  Defines the current position as zero steps.
    #  Used for defining the home position.
    def resetPosition(self):
        self.controller.ResetPos(self.motorNumber)
    ## @fn stop
    #  Commands the actuator to come to a soft stop.
    def stop(self):
        self.controller.softStop(self.motorNumber)
        
        
        