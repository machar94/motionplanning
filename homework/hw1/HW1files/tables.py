#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy

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
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot)


    #### YOUR CODE HERE ####

    ## 1. Move all tables to PR side ##
    with env:
        state = numpy.eye(4)
        state[0:3,0:3] = env.GetKinBody('Table1').GetTransform()[0:3,0:3]

        # Update the positioning of table with environment id 6
        state[0:3,3] = [-1.1, 0.9, 0.74]
        env.GetKinBody('Table5').SetTransform(state)

        # Update the positioning of table with environment id 7 
        state[0:3,3] = [-2.3, 0.9, 0.74]
        env.GetKinBody('Table6').SetTransform(state)

    #### END OF YOUR CODE ###



    raw_input("Press enter to exit...")
    env.Destroy()
