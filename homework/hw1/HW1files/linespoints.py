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

    # Keep handle to all graphics drawn (points and lines)
    # Reference to graphic needs to be kept alive for it to show up
    handles = []

    with env:

        # Identify all tables and for each table draw red box around border
        tables = env.GetBodies()[1:7]
        for table in tables:
            # Calculate bouunding box
            extent  = table.ComputeAABB().extents()
            pos     = table.ComputeAABB().pos()

            # Extract corners of bounding box
            top_right = pos + extent*array([1,1,1])
            top_left  = pos + extent*array([-1,1,1])
            bottom_right = pos + extent*array([1,-1,1])
            bottom_left = pos + extent*array([-1,-1,1])

            pointsArr = numpy.vstack((top_right, bottom_right, bottom_left, top_left, top_right))
        
            handles.append(env.drawlinestrip(points=pointsArr, linewidth=3.0, colors=array((1,0,0))))

    # Create a set of 35 blue points around the scene
    with env:
        numPoints = 35
        pointsArr = array([[5*cos(math.pi*2/numPoints*i), 
            5*sin(math.pi * 2 / numPoints * i), 0] for i in range(0,numPoints)])
        colorsArr = array([[0,0,1]]*numPoints)
        handles.append(env.plot3(points=pointsArr, pointsize=5.0, colors=colorsArr))

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()
