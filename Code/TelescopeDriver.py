# -*- coding: utf-8 -*-
#
## @file TelescopeDriver.py
#  This file contains a driver for a Three Parallel actuator telescope. 
#    
#
#
#  @author Samuel Steejans Artho-Bentz
import math
import time
import pyb

## @var NUMBER_OF_MICROSTEPS
#  Must define the number of microsteps to correct for oddity of stepper driver
NUMBER_OF_MICROSTEPS = 4

## @class Telescope
#  This class creates a telescope object controlled by 3 parallel actuators. 
#  Each actuator is driven by a stepper motor with an L6470 stepper driver.
#  @param actuatorOne Right actuator when viewed from the origin looking along the X axis
#  @param actuatorTwo Left actuator when viewed from the origin looking along the X axis
#  @param actuatorThree Rear, horizontal actuator
class Telescope(object):
    
    ## Initialize the Telescope object
    
    def __init__ (self, actuatorOne, actuatorTwo, actuatorThree):
    
        ## actuatorOne: Right actuator when viewed from the origin looking along the X axis
        self.actuatorOne = actuatorOne
        ## actuatorTwo: Left actuator when viewed from the origin looking along the X axis
        self.actuatorTwo = actuatorTwo
        ## actuatorThree: Rear, horizontal actuator
        self.actuatorThree = actuatorThree
                
        
        ## Origin with respect to the base
        self.P0_b = [[0],[0],[0]]    
        
        ## Fixed end of actuator one
        self.P1_b = [[-12],[-3.25],[16]]    
        
        ## Fixed end of actuator two
        self.P2_b = [[12],[-3.25],[16]]    
        
        ## Fixed end of actuator three
        self.P3_b = [[-12],[-3.25],[5]]    
        
        ## Optical Axis base
        self.OA_b = self.P0_b;                         


        ## Position of the moving end of actuator one in the home position
        self.P1_h = [[-13.55],[8.55],[10.64]]
        
        ## Position of the moving end of actuator two in the home position
        self.P2_h = [[6.85],[8.87],[15.63]]

        ## Position of the moving end of actuator three in the home position        
        self.P3_h = [[-1.32],[-2.44],[5.57]]

        ## Position of the moving end of optical axis in the home position        
        self.OA_h = [[-4.0],[3.75],[16.25]]
        
        # Calculate Minimum Lengths
        ## Minimum length of actuator one
        self.L_p1min = rootsumsquare(self.P1_h, self.P1_b)
        
        ## Minimum length of actuator two
        self.L_p2min = rootsumsquare(self.P2_h, self.P2_b)
        
        ## Minimum length of actuator three
        self.L_p3min = rootsumsquare(self.P3_h, self.P3_b)
        
        ## Fixed distance from the moving end of actuator one to actuator two
        self.L_p1p2  = rootsumsquare(self.P1_h, self.P2_h)
        
        #Calculate Correction Angles \phi to Rotate from home position to alt = 0, az = 0 
        
        ## The azimuth correction angle required to rotate from the home position to (0,0,0)
        self.phi_az = -math.atan2(self.OA_h[0][0], self.OA_h[2][0]);
        ## The altitude correction angle required to rotate from the home position to (0,0,0)
        self.phi_alt = math.atan2(self.OA_h[1][0], self.OA_h[2][0]);
        ## The image rotation correction angle required to rotate from the home position to (0,0,0)
        self.phi_rot = math.atan2(self.P1_h[1][0]-self.P2_h[1][0], self.L_p1p2)
        
        # Define current angles
        
        
        ## The current altitude of the system
        self.alt_cur = self.phi_alt
        ## The current azimuth of the system
        self.az_cur = self.phi_az
        ## The current image rotation of the system
        self.rot_cur = self.phi_rot
        
        # Print warning message
        print('Please do not interrupt any movement commands. Allow them to finish completely')
    
    ## @fn goVelocityAngular
    #  Move the telescope at specified radians/sec.
    #  @param omega_az Angular rate of azimuth (Rad/sec)
    #  @param omega_alt Angular rate of altitude (Rad/sec)
    #  @param omega_rot Angular rate of image rotation (Rad/sec)
    #  @param tstep time step for calculation of velocity. Default 1 sec
    def goVelocityAngular(self, omega_alt, omega_az, omega_rot, tstep=1):
        Walt = omega_alt
        Waz = omega_az
        Wrot = omega_rot
        print('Walt: ' +str(Walt)+ ' Waz: ' + str(Waz)+ ' Wrot: ' +str(Wrot))
        V = [0,0,0]
        # Use current position to find new target position
        vcp = pyb.USB_VCP () 
        while not vcp.any (): # causes any keypress to interrupt
            alt_new = self.alt_cur+Walt*tstep
            az_new = self.az_cur+Waz*tstep
            rot_new = self.rot_cur+Wrot*tstep
            L_new = self.goToAngles(alt_new, az_new, rot_new)
            print('New Angles are: ' + str(alt_new) + ' ' + str(az_new) + ' ' + str(rot_new))
            print('L_new is: ' + str(L_new))
            L1 = self.actuatorOne.getCurrentLength()+self.L_p1min
            L2 = self.actuatorTwo.getCurrentLength()+self.L_p2min
            L3 = self.actuatorThree.getCurrentLength()+self.L_p3min
            V[0] = (L_new[0]-(L1))/tstep/NUMBER_OF_MICROSTEPS 
            V[1] = (L_new[1]-(L2))/tstep/NUMBER_OF_MICROSTEPS
            V[2] = (L_new[2]-(L3))/tstep/NUMBER_OF_MICROSTEPS
            print('Current Lengths are: ' + str(L1) + ' ' + str(L2) + ' ' + str(L3))
            print('Velocities are: ' + str(V))
            self.actuatorOne.commandVelocity(V[0])
            self.actuatorTwo.commandVelocity(V[1])
            self.actuatorThree.commandVelocity(V[2])
            start = pyb.micros()
            self.alt_cur = alt_new
            self.az_cur = az_new
            self.rot_cur = rot_new
            while pyb.elapsed_micros(start)<int(tstep*(10**6)):
                pass
        self.actuatorOne.stop()
        self.actuatorTwo.stop()
        self.actuatorThree.stop()
        self.alt_cur = alt_new
        self.az_cur = az_new
        self.rot_cur = rot_new
        return ([self.alt_cur, self.az_cur, self.rot_cur])

    ## @fn goToAngles
    #  Function which calculates the correct lengths of the three linear actuators in order to
    #  achieve the specified angles (in degrees) then can send the scope to that location.
    #  @param O_alt Desired Altitude Angle in radians
    #  @param O_az Desired Azimuth Angle in radians
    #  @param O_rot Desired Rotation Angle in radians
    #  @param move Boolean value which determines if the function only calculates the required lengths or also commands motion
    def goToAngles(self, O_alt, O_az, O_rot, move=False):	

        # Assemble Translation Matrix from Base (b) reference frame to Telescope (s)
        # Rotations Azimuth -> Altitude -> Image Rotations
        sTb = matmul(RotZ(O_rot+self.phi_rot),matmul(RotX(-O_alt+self.phi_alt),RotY(-O_az+self.phi_az)))

        # Calculate Rotated Positions
        P1_r = matmul(sTb,self.P1_h)
        P2_r = matmul(sTb,self.P2_h)
        P3_r = matmul(sTb,self.P3_h)
        OA_r = matmul(sTb,self.OA_h)
        
        # Calculate Length
        #L_p1 = pow(pow((P1_r[0][0]-self.P1_b[0][0]),2)+pow((P1_r[1][0]-self.P1_b[1][0]),2)+pow((P1_r[2][0]-self.P1_b[2][0]),2),.5)
        #L_p2 = pow(pow((P2_r[0][0]-self.P2_b[0][0]),2)+pow((P2_r[1][0]-self.P2_b[1][0]),2)+pow((P2_r[2][0]-self.P2_b[2][0]),2),.5)
        #L_p3 = pow(pow((P3_r[0][0]-self.P3_b[0][0]),2)+pow((P3_r[1][0]-self.P3_b[1][0]),2)+pow((P3_r[2][0]-self.P3_b[2][0]),2),.5)
        L_p1 = rootsumsquare(P1_r, self.P1_b)
        L_p2 = rootsumsquare(P2_r, self.P2_b)
        L_p3 = rootsumsquare(P3_r, self.P3_b)
        # Need to check that the lengths are legitimate
        if abs(L_p1<self.L_p1min) or abs(L_p2<self.L_p2min) or abs(L_p3<self.L_p3min):
            print('This position is beyond the minimum lengths')
        # Need to move actuators
        elif move == True:
            self.goToLengths(L_p1-self.L_p1min, L_p2-self.L_p2min, L_p3-self.L_p3min)
                    
            # Update current angles 
            self.alt_cur = O_alt
            self.az_cur = O_az
            self.rot_cur = O_rot
        return(L_p1, L_p2, L_p3) 
    
    
    ## @fn findHome
    #  Causes each actuator to shorten until it stalls then stops
    #  @b WARNING: Does not function!
    def findHome(self):
        '''
        Non-functioning function which is designed to implement automatic home checking
        '''
        isOneHome = False
        isTwoHome = False
        isThreeHome = False
        self.actuatorOne.findHome()
        self.actuatorTwo.findHome()
        self.actuatorThree.findHome()
        
        while not(isOneHome and isTwoHome and isThreeHome):
            if self.actuatorOne.isHome():
                print('A1 home')
                isOneHome = True
            if self.actuatorTwo.isHome():
                isTwoHome = True
                print('A2 home')
            if self.actuatorThree.isHome():
                isThreeHome = True
                print('A3 home')


            time.sleep(.01)

        print("There's no place like home")

    ## @fn resetPositions
    #  Zeros the absolute position of each actuator.
    #  Used for defining the home position.
    def resetPositions(self):

        self.actuatorOne.resetPosition()
        self.actuatorTwo.resetPosition()
        self.actuatorThree.resetPosition()
        
    ## @fn goToLengths
    #  Commands each actuator to a specified length
    #  @param lengthOne Desired length for actuator one
    #  @param lengthTwo Desired length for actuator two
    #  @param lengthThree Desired length for actuator three
    def goToLengths(self, lengthOne, lengthTwo, lengthThree):

        self.actuatorOne.commandLength(lengthOne)
        self.actuatorTwo.commandLength(lengthTwo)
        self.actuatorThree.commandLength(lengthThree)
        
    ## @fn goVelocity
    #  Commands each actuator to move at a specified velocity
    #  @param velOne Velocity in length/sec
    #  @param velTwo Velocity in length/sec
    #  @param velThree Velocity in length/sec
    def goVelocity(self, velOne, velTwo, velThree):

        self.actuatorOne.commandVelocity(velOne)
        self.actuatorTwo.commandVelocity(velTwo)
        self.actuatorThree.commandVelocity(velThree)
        
        
## Calculates the root sum square of two vectors of size 3
#  @param a vector of size 3
#  @param b vector of size 3
#  @return The root-sum-of-squares of the difference of @c a and @c b
def rootsumsquare(a, b):
    rss = pow(pow((a[0][0]-b[0][0]),2)+pow((a[1][0]-b[1][0]),2)+pow((a[2][0]-b[2][0]),2),.5)
    return rss
    
## Multiplies two matrices together @c a * @c b
#  @param a [m x n] matrix
#  @param b [n x o] matrix
def matmul(a, b):
    zip_b = zip(*b)
    zip_b = list(zip_b)
    return [[sum(ele_a*ele_b for ele_a, ele_b in zip(row_a, col_b)) for col_b in zip_b] for row_a in a]

## Rotation matrix about the x axis
def RotX(rads):
    return [[1,0,0],[0,math.cos(rads),-math.sin(rads)],[0,math.sin(rads),math.cos(rads)]]

## Rotation matrix about the y axis
def RotY(rads):
    return [[math.cos(rads),0,math.sin(rads)],[0,1,0],[-math.sin(rads),0,math.cos(rads)]] 

## Rotation matrix about the z axis
def RotZ(rads):
    return [[math.cos(rads),-math.sin(rads),0],[math.sin(rads),math.cos(rads),0],[0,0,1]]

"""
def getVelocity(Theta_x, Theta_y, Theta_z, Omega_x, Omega_y, Omega_z):
    ''' 
    This function calculates the time derivative of a rotation matrix defined by 
    RotZ(Theta_z)*RotX(Theta_x)*RotY(Theta_y)
    Each theta has an associated angular velocity omega. All angles are in radians
    '''
    sOx = math.sin(Theta_x)
    cOx = math.cos(Theta_x)
    sOy = math.sin(Theta_y)
    cOy = math.cos(Theta_y)
    sOz = math.sin(Theta_z)
    cOz = math.cos(Theta_z)
    Wx = Omega_x
    Wy = Omega_y
    Wz = Omega_z
    a11 = Wx*(-sOz*cOx*sOy)+Wy*(-cOz*sOy-sOz*sOx*cOy)+Wz*(-sOz*cOy-cOz*sOx*sOy)
    a12 = Wx*(sOz*sOx)+Wy*(0)+Wz*(-cOz*cOx)
    a13 = Wx*(sOz*cOx*cOy)+Wy*(cOz*cOy-sOz*sOx*sOy)+Wz*(-sOz*sOy+cOz*sOx*cOy)
    a21 = Wx*(cOz*cOx*sOy)+Wy*(-sOz*sOy+cOz*sOx*cOy)+Wz*(cOz*cOy-sOz*sOx*sOy)
    a22 = Wx*(-cOz*sOx)+Wy*(0)+Wz*(-sOz*cOx)
    a23 = Wx*(-cOz*cOx*cOy)+Wy*(sOz*sOy+cOz*sOx*sOy)+Wz*(cOz*sOy+sOz*sOx*cOy)
    a31 = Wx*(sOx*sOy)+Wy*(-cOx*cOy)+Wz*(0)
    ''' double check next line'''
    a32 = Wx*(cOx)+Wy*(0)+Wz*(0) 
    a33 = Wx*(-sOx*cOy)+Wy*(-cOx*sOy)+Wz*(0)
    return [[a11,a12,a13],[a21,a22,a23],[a31,a32,a33]]
"""