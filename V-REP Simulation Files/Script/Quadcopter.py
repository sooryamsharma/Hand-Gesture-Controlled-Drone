# -*- coding: utf-8 -*-
"""
Created on Thu Feb 14 15:57:14 2019

@author: Shashank
"""

import vrep
import sys
import math
import time
#from pynput.keyboard import Key, Listener

clientID=0
velocityX = 0.0
velocityY = 0.0
height = 0.0
Quadricopter_target = 'Quadricopter_target'
Quadricopter_base = 'Quadricopter_base'
Quadricopter = 'Quadricopter'
qc_handle = 0
targetObj = 0
debug = True

#def on_press(event):
#    [returnCode, qc_position] = vrep.simxGetObjectPosition(clientID, qc_handle,-1,vrep.simx_opmode_blocking)
#    if event == Key.up:
#        if debug:
#            print("UP")
#            #[returnCode, qc_position] = vrep.simxGetObjectPosition(clientID, qc_handle,-1,vrep.simx_opmode_blocking)
#            vrep.simxSetObjectPosition(clientID, -1, qc_handle, (qc_position[0], qc_position[1] + 1, qc_position[2]),  vrep.simx_opmode_blocking)
#            vrep.simxSynchronousTrigger(clientID)
#            #vrep.simxSetJointTargetVelocity(clientID, RightMotor, Speed, vrep.simx_opmode_blocking)
#            #print '--------------------------------- Direction Straight'
#    elif event == Key.right:
#        if debug:
#            print( "Right")
#            vrep.simxSetObjectPosition(clientID, -1, qc_handle, (qc_position[0] + 1, qc_position[1], qc_position[2]), vrep.simx_opmode_blocking)
#            vrep.simxSynchronousTrigger(clientID)
#            #vrep.simxSetJointTargetVelocity(clientID, RightMotor, -Speed, vrep.simx_opmode_blocking)
#            #print '--------------------------------- Direction turn right'
#    elif event == Key.left:
#        if debug:
#            print( "Left")
#            vrep.simxSetObjectPosition(clientID, -1, qc_handle, (qc_position[0] - 1, qc_position[1], qc_position[2]), vrep.simx_opmode_blocking)
#            vrep.simxSynchronousTrigger(clientID)
#            #vrep.simxSetJointTargetVelocity(clientID, LeftMotor, -Speed, vrep.simx_opmode_blocking)
#            #print '--------------------------------- Direction turn left'
#    elif event == Key.down:
#        if debug:
#            print( "down")
#            vrep.simxSetObjectPosition(clientID, -1, qc_handle, (qc_position[0], qc_position[1] - 1, qc_position[2]), vrep.simx_opmode_blocking)
#            vrep.simxSynchronousTrigger(clientID)
#            #vrep.simxSetJointTargetVelocity(clientID, RightMotor, -Speed, vrep.simx_opmode_blocking)
#            #print '--------------------------------- Direction turn Back'
#
#def on_release(key):
#    if debug:
#        print("{} released".format(key))
#        [returnCode, qc_position] = vrep.simxGetObjectPosition(clientID, qc_handle,-1,vrep.simx_opmode_blocking)
#        vrep.simxSetObjectPosition(clientID, targetObj, qc_handle, qc_position, vrep.simx_opmode_blocking)
#        vrep.simxSetJointTargetVelocity(clientID, targetObj, qc_handle, qc_position, vrep.simx_opmode_blocking)

def movement(clientID,direction):
  
    [returnCode,targetObj]=vrep.simxGetObjectHandle(clientID,Quadricopter_target,vrep.simx_opmode_blocking)
    [returnCode,qc_base_handle]=vrep.simxGetObjectHandle(clientID,Quadricopter_base,vrep.simx_opmode_blocking)
    [returnCode,qc_handle]=vrep.simxGetObjectHandle(clientID,Quadricopter,vrep.simx_opmode_blocking)
    [returnCode, target_position] = vrep.simxGetObjectPosition(clientID, targetObj,-1,vrep.simx_opmode_blocking)

    #  Collect events until released
    initial_position = target_position
    prev_direction = ""
    
    if direction == "right" and prev_direction != "right":
         flag = True
         prev_direction = "right"
         while flag:
            vrep.simxSetJointTargetVelocity(clientID,targetObj,1,vrep.simx_opmode_oneshot)
            vrep.simxSetObjectPosition(clientID, targetObj, -1, (target_position[0] + 2, 0, initial_position[2]), vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            [returnCode, base_position] = vrep.simxGetObjectPosition(clientID, qc_base_handle,-1,vrep.simx_opmode_blocking)
            [returnCode, current_position] = vrep.simxGetObjectPosition(clientID, targetObj,-1,vrep.simx_opmode_blocking)
            if math.floor(base_position[0]) == math.floor(current_position[0]):
                flag = False
    
            
    elif direction == "left" and prev_direction != "left":
        [returnCode, target_position] = vrep.simxGetObjectPosition(clientID, targetObj,-1,vrep.simx_opmode_blocking)
        flag = True
        prev_direction = "left"
        while flag:
            vrep.simxSetJointTargetVelocity(clientID,targetObj,2,vrep.simx_opmode_oneshot)
            vrep.simxSetObjectPosition(clientID, targetObj, -1, (target_position[0] - 2, 0, initial_position[2]), vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            [returnCode, base_position] = vrep.simxGetObjectPosition(clientID, qc_base_handle,-1,vrep.simx_opmode_blocking)
            [returnCode, current_position] = vrep.simxGetObjectPosition(clientID, targetObj,-1,vrep.simx_opmode_blocking)
            if math.floor(base_position[0]) == math.floor(current_position[0]):
                flag = False
  

def initialize():
        global clientID
        
        vrep.simxFinish(-1)
        clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
             # Connect to V-REP
        if clientID !=-1:
            print ('Connected to remote API server')
            res, v1 = vrep.simxGetObjectHandle(clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)
            print(res)
            rc,sv=vrep.simxGetFloatSignal(clientID,'particlesTargetVelocities',vrep.simx_opmode_oneshot_wait)
            print(rc,sv)
            # enable the synchronous mode on the client:
            vrep.simxSynchronous(clientID,True)

            # load the quadroter model
            vrep.simxLoadModel(clientID,"C:\quadrotor.ttm",0,vrep.simx_opmode_blocking)

            # start the simulation:
            vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)
            #functional/handle code:

#                with Listener(
#                        on_press=on_press,
#                        on_release=on_release) as listener:
#                        listener.join()
            return clientID
        else: 
            print('Not Connected')
            sys.exit('Not Connected')
            return -1

if __name__ == "__main__":
    direction = "right"
    clientID = initialize();
    if(clientID != -1):
      movement(clientID,direction)
      
      