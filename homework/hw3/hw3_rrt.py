#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import IPython
import numpy as np
import matplotlib.pyplot as plt
import os
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

def plotStatistics(times, nodes, samples, goalbias, pathL, smooth, smoothT, nspathl):
    print "\n\n===== Summary of Simulations ====="
    print "Average Time    : %f" % (np.average(times))
    print "Average Samples : %f" % (np.average(samples))
    print "Average Nodes   : %f" % (np.average(nodes))

    FILENAME = 'goalBias.txt'

    try:
        os.remove(FILENAME)
    except OSError:
        pass

    plt.rcParams.update({'font.size': 22})
    np.set_printoptions(precision=2)
    np.set_printoptions(suppress=True)

    f_handle = file(FILENAME, 'a')
    np.savetxt(f_handle, goalbias, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, times, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, nodes, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, samples, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, smooth, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pathL, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, smoothT, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, nspathl, delimiter=',', fmt='%1.2f')
    f_handle.close()
    

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
    RaveLoadPlugin('rrtconnect/build/rrtconnect')
    RRTConnect = RaveCreateModule(env,'RRTConnect')
    env.AddModule(RRTConnect,args='')
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
        NUM_SAMPLES = 10000
        GOAL_BIAS_VAL = 11     # int value between (0, 100]
        STEP_SIZE = 0.05
        smoothing = 200;

        # Register the start configuration
        startConfigStr = ' '.join([str(e) for e in startconfig])
        RRTConnect.SendCommand('setstart ' + startConfigStr)

        # Register the goal configuration
        goalConfigStr = ' '.join([str(e) for e in goalconfig])
        RRTConnect.SendCommand('setgoal ' + goalConfigStr)

        # Set the number of samples generated by the RRT before timing out
        RRTConnect.SendCommand('setnumsamples ' + str(NUM_SAMPLES))

        # Set goal biasing percentage (percentabe = 1 / biasVal)
        RRTConnect.SendCommand('setgoalbias ' + str(GOAL_BIAS_VAL))

        # Set step size
        RRTConnect.SendCommand('setstepsize ' + str(STEP_SIZE))

        RRTConnect.SendCommand('init')
        # RRTConnect.SendCommand('printclass')

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
        RRTConnect.SendCommand('resettree')
        RRTConnect.SendCommand('setgoalbias ' + str(GOAL_BIAS_VAL))
        result = RRTConnect.SendCommand('run')

        # data    = [double(val) for val in result.split()]
        # times   = np.append(times, [[data[0]]], axis=0)
        # nodes   = np.append(nodes, [[data[1]]], axis=0)
        # samples = np.append(samples, [[data[2]]], axis=0)
        # goalBL  = np.append(goalBL, [[GOAL_BIAS_VAL]], axis=0)
        # pathL   = np.append(pathL, [[data[3]]], axis=0)
        # smooth  = np.append(smooth, [[smoothing]], axis=0)
        # smoothT = np.append(smoothT, [[data[4]]], axis=0)
        # nspathl = np.append(nspathl, [[data[5]]], axis=0)


        ###############################
        # Test for multiple runs
        ###############################
        # numLoops = 10;
        # for i in xrange(0,numLoops):
        #     RRTConnect.SendCommand('resettree')
        #     RRTConnect.SendCommand('setgoalbias ' + str(GOAL_BIAS_VAL))
        #     result = RRTConnect.SendCommand('run')

        #     data = [double(val) for val in result.split()]
        #     times = np.append(times, [[data[0]]], axis=0)
        #     nodes = np.append(nodes, [[data[1]]], axis=0)
        #     samples = np.append(samples, [[data[2]]], axis=0)

        ###############################
        # Test for various goal biasing
        ###############################

        # goalbias = 66
        # numLoops = 10;
        # while goalbias < 100:
            
        #     print "\n\n================================="
        #     print "===== Evaluating Goal Bias %d =====" % goalbias

        #     times_avg    = np.empty(shape=[0,1])
        #     samples_avg  = np.empty(shape=[0,1])
        #     nodes_avg    = np.empty(shape=[0,1])

        #     for i in xrange(0,numLoops):
        #         RRTConnect.SendCommand('resettree')
        #         RRTConnect.SendCommand('setgoalbias ' + str(goalbias))
        #         result = RRTConnect.SendCommand('run')

        #         data = [double(val) for val in result.split()]
        #         times_avg   = np.append(times_avg, [[data[0]]], axis=0)
        #         nodes_avg   = np.append(nodes_avg, [[data[1]]], axis=0)
        #         samples_avg = np.append(samples_avg, [[data[2]]], axis=0)
            
        #     times = np.append(times, [[np.average(times_avg)]], axis=0)
        #     nodes = np.append(nodes, [[np.average(nodes_avg)]], axis=0)
        #     samples = np.append(samples, [[np.average(samples_avg)]], axis=0)
        #     goalBL = np.append(goalBL, [[goalbias]], axis=0)

        #     goalbias = goalbias + 5

        ###############################
        # Test for smoothing
        ###############################
        # Todo: Also fix srand in rrtconnect.cpp
        # while smoothing < 300:
        #     RRTConnect.SendCommand('resettree')
        #     RRTConnect.SendCommand('setsmoothiteration ' + str(smoothing))
        #     result = RRTConnect.SendCommand('run')

        #     data    = [double(val) for val in result.split()]
        #     times   = np.append(times, [[data[0]]], axis=0)
        #     nodes   = np.append(nodes, [[data[1]]], axis=0)
        #     samples = np.append(samples, [[data[2]]], axis=0)
        #     goalBL  = np.append(goalBL, [[GOAL_BIAS_VAL]], axis=0)
        #     pathL   = np.append(pathL, [[data[3]]], axis=0)
        #     smooth  = np.append(smooth, [[smoothing]], axis=0)
        #     smoothT = np.append(smoothT, [[data[4]]], axis=0)
        #     nspathl = np.append(nspathl, [[data[5]]], axis=0)
            
        #     smoothing = smoothing + 20

        ###############################
        # Data collection for HW Pt 5
        ###############################
        # numLoops = 30;
        # for i in xrange(0,numLoops):
        #     RRTConnect.SendCommand('resettree')
        #     RRTConnect.SendCommand('setgoalbias ' + str(GOAL_BIAS_VAL))
        #     result = RRTConnect.SendCommand('run')

        #     data    = [double(val) for val in result.split()]
        #     times   = np.append(times, [[data[0]]], axis=0)
        #     nodes   = np.append(nodes, [[data[1]]], axis=0)
        #     samples = np.append(samples, [[data[2]]], axis=0)
        #     goalBL  = np.append(goalBL, [[GOAL_BIAS_VAL]], axis=0)
        #     pathL   = np.append(pathL, [[data[3]]], axis=0)
        #     smooth  = np.append(smooth, [[smoothing]], axis=0)
        #     smoothT = np.append(smoothT, [[data[4]]], axis=0)
        #     nspathl = np.append(nspathl, [[data[5]]], axis=0)

        # plotStatistics(
        #     np.transpose(times),
        #     np.transpose(nodes),
        #     np.transpose(samples),
        #     np.transpose(goalBL), 
        #     np.transpose(pathL), 
        #     np.transpose(smooth), 
        #     np.transpose(smoothT),
        #     np.transpose(nspathl))
 
        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

