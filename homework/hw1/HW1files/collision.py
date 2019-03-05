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
    tuckarms(env,robot);


    #### YOUR CODE HERE ####

    # Place robot in front of wall
    state = numpy.eye(4)
    state[2,3] = 0.05

    with env:
        robot.SetTransform(state)
        print "After placing robot in front of wall, is it colliding with anything? ", env.CheckCollision(robot)

        # Set right arm joints active
        robot.SetActiveDOFs(range(27,34))

        # Extend right arm joints
        config = robot.GetActiveDOFValues()
        config[1] = 0.0                 # Set the shoulder lift joint
        config[3] = math.radians(-20)   # Set the elbow
        config[5] = math.radians(-20)   # Set the wrist flex

        robot.SetActiveDOFValues(config)
        print "After extending arm, is it colliding with the wall? ", env.CheckCollision(robot)

        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()
