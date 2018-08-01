# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 11:07:58 2017

@author: Artho-Bentz, Samuel
"""

class Actuator(object):
    ''' 
    @details Object which represents one linear actuator for a 3 DOF parallel actuator telescope mount
    @param stepperDriver stepper driver object which powers the actuator
    @param stepperResolution Resolution of stepper motor in steps / revolution
    @param rodResolution Resolution of actuator rod in inches / revolution
    '''
    
    def __init__(self, stepPerLength, motorController, motorNumber):
        self.stepPerLength = stepPerLength
        self.controller = motorController
        self.motorNumber = motorNumber
        self.currentLength = 0

    def commandVelocity(self, velocity):
        '''
            @param velocity length/sec
        '''
        stepsPerSec = self.__calcSteps(velocity)
        print('Command to ' + str(stepsPerSec) + ' steps/sec')
        self.controller.run(self.motorNumber, stepsPerSec)
        
    def commandLength(self, length):
        steps = self.__calcSteps(length)
        print(['command to ' + str(steps) + ' steps'])
        self.__commandStepMotion(steps)        
        
    def __calcSteps(self, length):
        steps = round(length*self.stepPerLength)
        return steps
        
    def __commandStepMotion(self,steps):
        self.controller.GoTo(self.motorNumber, steps)
    
    def getCurrentLength(self):
        self.currentLength = self.controller.getPositions(self.motorNumber)/self.stepPerLength
        return self.currentLength
        
    def findHome(self):
        self.controller.run(self.motorNumber, -10)
        
    def isHome(self):
        stalled = self.controller.isStalled(self.motorNumber, True)
        if stalled:
            self.controller.HardHiZ(self.motorNumber)
            return True
        else:
            return False
            
    def resetPosition(self):
        self.controller.ResetPos(self.motorNumber)
    def stop(self):
        self.controller.softStop(self.motorNumber)
        
        
        