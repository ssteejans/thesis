# -*- coding: utf-8 -*-
#
## @file main.py
#  Control file for a Three Degree of Freedom Parallel Actuator Telescope Mount
#    
#
#
#  @author Samuel Steejans Artho-Bentz
import pyb
import l6470nucleo
import time
import ActuatorDriver
import TelescopeDriver




## Stepper Resolution [steps/rev]
STEPPER_RESOLUTION = const (200)

## Number of microsteps
#  This must match @c NUMBER_OF_MICROSTEPS in @c TelescopeDriver.py and 
#  @c STEP_SEL in @c l6470nucleo.py 
MICROSTEPS = 4

## Leadscrew Resolution [rev/in]
SCREW_RESOLUTION = const(16)

# Set up pins

## SPI one standby/reset pin
stby_rst_pin1 = pyb.Pin.cpu.B5
## SPI two standyby/reset pin
stby_rst_pin2 = pyb.Pin.cpu.B3
## SPI one chip select pin
nCS1 = pyb.Pin.cpu.A4
## SPI two chip select pin
nCS2 = pyb.Pin.cpu.A10
## SPI number
spi_number = 1
print('pins initialized')

## spi object for motor drivers
spi_object = pyb.SPI (spi_number, mode=pyb.SPI.MASTER, 
                           baudrate=2000000, polarity=1, phase=1, 
                           bits=8, firstbit=pyb.SPI.MSB)
                           
## l6470nucleo board object
Driver1 = l6470nucleo.Dual6470(spi_object,nCS1, stby_rst_pin1)
## l6470nucleo board object
Driver2 = l6470nucleo.Dual6470(spi_object,nCS2, stby_rst_pin2)

Driver1._get_params(0xD0,2)

# Define the 3 actuators
## Right actuator when viewed from the origin looking along the X axis
actuatorOne = ActuatorDriver.Actuator(STEPPER_RESOLUTION*SCREW_RESOLUTION*MICROSTEPS, Driver1, 1)
## Left actuator when viewed from the origin looking along the X axis
actuatorTwo = ActuatorDriver.Actuator(STEPPER_RESOLUTION*SCREW_RESOLUTION*MICROSTEPS, Driver1, 2)
## Rear, horizontal actuator
actuatorThree = ActuatorDriver.Actuator(STEPPER_RESOLUTION*SCREW_RESOLUTION*MICROSTEPS, Driver2, 1)

## Define the telescope based on the actuators above
Telescope = TelescopeDriver.Telescope(actuatorOne, actuatorTwo, actuatorThree)


# Create some basic utility functions

## @fn estop
#  commands all motors to halt immediately.
def estop():
   Driver1.HardHiZ(1)
   Driver1.HardHiZ(2)
   Driver2.HardHiZ(1)

## @fn Go(speed)
#  commands all motors to go at the specified speed.
#  directly communicates with the stepper motors.
#  @param speed steps/second
def Go(speed):
   Driver1.run(1, speed)
   Driver1.run(2, speed)
   Driver2.run(1, speed)

## @fn getPos
#  Queries the actuators for their current lengths
def getPos():
   print(['Actuator One is at: ' + str(actuatorOne.getCurrentLength()+Telescope.L_p1min)])
   print(['Actuator Two is at: ' + str(actuatorTwo.getCurrentLength()+Telescope.L_p2min)])
   print(['Actuator Three is at: ' + str(actuatorThree.getCurrentLength()+Telescope.L_p3min)])

## @fn getStatus(motornumber)
#  Requests the status of the specified motor
#  @param motornumber The motor associated with the desired actuator: 1, 2, or 3
def getStatus(motornumber):
   if motornumber ==1:
       actuatorOne.controller.GetStatus(1, 1)
   elif motornumber ==2:
       actuatorTwo.controller.GetStatus(2, 1)
   elif motornumber ==3:
       actuatorThree.controller.GetStatus(1, 1)

## @fn testRepeatability(numloops)
#  Runs through a series of points @c numloops times.
#  Waits for the user to hit a key before moving on to the next point.
#  Used for testing.
#  @param numloops Desired number of times the points should be gone through.
def testRepeatability(numloops=10):

  pt1 = [.349066, 0, 0]
  pt2 = [.523599, -0.17453, 0]
  pt3 = [0.698132, -0.34907, 0]
  pt4 = [0.872665, -0.17453, 0]
  pt5 = [0.959931, 0, 0]
  pt6 = [0.872665, 0.174533, 0]
  pt7 = [0.785398, .349066, 0]
  pt8 = [0.523599, 0.174533, 0]
  Telescope.goToAngles(pt1[0], pt1[1], pt1[2], 1)
  print('point 1')
  input()
  Telescope.goToAngles(pt2[0], pt2[1], pt2[2], 1)
  print('point 2')
  input()
  Telescope.goToAngles(pt3[0], pt3[1], pt3[2], 1)
  print('point 3')
  input()
  Telescope.goToAngles(pt4[0], pt4[1], pt4[2], 1)
  print('point 4')
  input()
 
  Telescope.goToAngles(pt6[0], pt6[1], pt6[2], 1)
  print('point 6')
  input()
  Telescope.goToAngles(pt7[0], pt7[1], pt7[2], 1)
  print('point 7')
  input()
  Telescope.goToAngles(pt8[0], pt8[1], pt8[2], 1)
  print('point 8')
  input()
  for x in range(1,numloops):
    Telescope.goToAngles(pt1[0], pt1[1], pt1[2], 1)
    print('going to 20,0')
    time.sleep(80)
    Telescope.goToAngles(pt2[0], pt2[1], pt2[2], 1)
    print('going to 30,-10')
    time.sleep(80)
    Telescope.goToAngles(pt3[0], pt3[1], pt3[2], 1)
    print('going to 40,-20')
    time.sleep(80)
    Telescope.goToAngles(pt4[0], pt4[1], pt4[2], 1)
    print('going to 50, ,0')
    time.sleep(80)
    Telescope.goToAngles(pt6[0], pt6[1], pt6[2], 1)
    print('going to 50,10')
    time.sleep(80)
    Telescope.goToAngles(pt7[0], pt7[1], pt7[2], 1)
    print('going to 45,20')
    time.sleep(80)
    Telescope.goToAngles(pt8[0], pt8[1], pt8[2], 1)
    print('going to 30,10')
    time.sleep(100)
  input()
## @fn testGrid
#  Moves to a series of points and waits for user input at each one.
#  Used for testing.
def testGrid():
  alts = [.296705973, .34906585, .436332313, .523598776, .610865238]
  azs = [.174532925, .087266463, 0, -.087266463, -.174532925]
  for x in azs:
    for y in alts:
      Telescope.goToAngles(y, x, 0,1)
      print('Altitude: ', str(y), ' Azimuth: ', str(x))
      input()