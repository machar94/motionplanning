#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import os
import IPython
import numpy as np

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveLoadPlugin('birrt/build/birrt')
    BIRRT = RaveCreateModule(env,'BiRRT')
    env.AddModule(BIRRT,args='')
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    tuckarms(env,robot)
  
    #set start config
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      
    startconfig = [-0.15,0.075,-1.008,0,0,-0.11,0]
    robot.SetActiveDOFValues(startconfig)
    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    with env:
        goalconfig = [0.449,-0.201,-0.151,0,0,-0.11,0]    # Actual goal
        # goalconfig = [0.5255,1.29,-2.12,0,0,-1.38,0]      # level 4 goal
        # goalconfig = [0.5255,1.29,-2.12,0,0,-0.11,0]      # level 3 goal
        # goalconfig = [-0.15,1.29,-2.12,0,0,-1.38,0]       # level 2 goal
        # goalconfig = [-0.15,-0.35,-1.73,0,0,-0.11,0]      # level 1 goal

        ### YOUR CODE HERE ###
        STEP_SIZE     = 0.05
        SMOOTHING     = 200
        SEED          = 248

        # Set parameters for BIRRT
        startConfigStr = ' '.join([str(e) for e in startconfig])
        goalConfigStr = ' '.join([str(e) for e in goalconfig])

        BIRRT.SendCommand('setstart ' + startConfigStr)
        BIRRT.SendCommand('setgoal ' + goalConfigStr)
        BIRRT.SendCommand('setstepsize ' + str(STEP_SIZE))
        # BIRRT.SendCommand('setrandomseed ' + str(SEED))
        BIRRT.SendCommand('init')
        BIRRT.SendCommand('printclass')


        times    = np.empty(shape=[0,1])
        samples  = np.empty(shape=[0,1])
        nodes    = np.empty(shape=[0,1])
        goalBL   = np.empty(shape=[0,1])
        pathL    = np.empty(shape=[0,1])
        smooth   = np.empty(shape=[0,1])
        smoothT  = np.empty(shape=[0,1])
        nspathl  = np.empty(shape=[0,1])

        ###############################
        # Test for various single test
        ###############################
        result = BIRRT.SendCommand('run')
        #     data = [double(val) for val in result.split()]
        #     times = np.append(times, [[data[0]]], axis=0)
        #     nodes = np.append(nodes, [[data[1]]], axis=0)
        #     samples = np.append(samples, [[data[2]]], axis=0)
        #     goalBL = np.append(goalBL, [[GOAL_BIAS_VAL]], axis=0)

        ### END OF YOUR CODE ###
    
    waitrobot(robot)

    raw_input("Press enter to exit...")

