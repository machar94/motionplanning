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

    env.Load('robots/puma.robot.xml')
    puma = env.GetRobot('PumaGripper')
    pr2 = env.GetRobot('PR2')
    
    # Place Puma gripper in front of pr2
    state = numpy.eye(4)
    state[0:3,3] = [-2.63536, -1.48728, 0.02580]

    with env:
        puma.SetTransform(state)

        # Extend the left arm of the PR2 robot
        joints = pr2.GetJoints()
        leftArmJointIndices = pr2.GetActiveDOFIndices()[:3]
        leftArmJoints = [joints[i] for i in leftArmJointIndices]

        lShoulder = leftArmJoints[0]
        pr2.SetJointValues([0], [lShoulder.GetDOFIndex()])

        lElbow = leftArmJoints[1]
        pr2.SetJointValues([-1], [lElbow.GetDOFIndex()])

        # Check for collision
        print "Collision between Puma and PR2? ", env.CheckCollision(pr2, puma)
    
    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()
