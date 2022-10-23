import time
from arm import AcrobotEnv
import iteratenums as confspace
import PathPlanner as plan
import sys
import math
import copy
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':


    class Pid:
        def __init__(self, Kp, Ki, Kd):####### do I need 2 Kps and 2 Kds, etc since it is two arms??
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.error = 0
            self.previousError = 0
            self.changeInError = 0
            self.cummulativeError = 0

        def action(self, timeStep):
            motorCommand = self.Kp*self.error+(self.changeInError)*self.Kd+self.Ki*self.cummulativeError# Step 2: Write a PID controller for motor speed
            return motorCommand

    def anglediff(goal,current):
        error= math.atan2(math.sin(goal-current), math.cos(goal-current))
        return error

    def sign(v_err):
        if v_err <= 0:
            return -1
        return 1

    arm = AcrobotEnv() # set up an instance of the arm class
    arm.rendering = True # True displays the video

    timeStep = 0.02 # sec
    timeForEachMove = 0.3 # sec
    stepsForEachMove = round(timeForEachMove/timeStep)

    # Make configuraton space
    world,crash = confspace.getConfigSpace()
##    np.set_printoptions(threshold=sys.maxsize)
##    print(world)

    # Get three waypoints from the user
    Ax = int(input("Type Ax: "))
    Ay = int(input("Type Ay: "))
    Bx = int(input("Type Bx: "))
    By = int(input("Type By: "))
    Cx = int(input("Type Cx: "))
    Cy = int(input("Type Cy: "))


    arm.Ax = Ax*0.0254; # Simulaiton is in SI units
    arm.Ay = Ay*0.0254; # Simulaiton is in SI units
    arm.Bx = Bx*0.0254; # Simulaiton is in SI units
    arm.By = By*0.0254; # Simulaiton is in SI units
    arm.Cx = Cx*0.0254; # Simulaiton is in SI units
    arm.Cy = Cy*0.0254; # Simulaiton is in SI units

    # Inverse Kinematics
    # 6 sets of angles, 2 sets for each coordinate (A,B,C) for the two solutions for that point
    # (A_base1, A_joint1), (A_base2, A_joint2), (B_base1, B_joint1), ...
    L1 = 3.75*0.0254 #inches
    L2 = 2.5*0.0254 #inches

    A_joint1 = np.arccos((arm.Ax**2 + arm.Ay**2 - L1**2 - L2**2)/(2*L1*L2))
    A_base1 = np.arctan2(arm.Ay, arm.Ax) - np.arcsin((L2*np.sin(A_joint1))/np.sqrt(arm.Ax**2 + arm.Ay**2))
    A_joint2 = (2*np.pi) - A_joint1
    A_base2 = np.arctan2(arm.Ay, arm.Ax) - np.arcsin((L2*np.sin(A_joint2))/np.sqrt(arm.Ax**2 + arm.Ay**2))

    B_joint1 = np.arccos((arm.Bx**2 + arm.By**2 - L1**2 - L2**2)/(2*L1*L2))
    B_base1 = np.arctan2(arm.By, arm.Bx) - np.arcsin((L2*np.sin(B_joint1))/np.sqrt(arm.Bx**2 + arm.By**2))
    B_joint2 = (2*np.pi) - B_joint1
    B_base2 = np.arctan2(arm.By, arm.Bx) - np.arcsin((L2*np.sin(B_joint2))/np.sqrt(arm.Bx**2 + arm.By**2))

    C_joint1 = np.arccos((arm.Cx**2 + arm.Cy**2 - L1**2 - L2**2)/(2*L1*L2))
    C_base1 = np.arctan2(arm.Cy, arm.Cx) - np.arcsin((L2*np.sin(C_joint1))/np.sqrt(arm.Cx**2 + arm.Cy**2))
    C_joint2 = (2*np.pi) - C_joint1
    C_base2 = np.arctan2(arm.Cy, arm.Cx) - np.arcsin((L2*np.sin(C_joint2))/np.sqrt(arm.Cx**2 + arm.Cy**2))

    #Plan Path Here
    waypoints = np.array([np.array([int(A_base1/(np.pi/300)), int(A_joint1/(np.pi/300))]),
                          np.array([int(B_base1/(np.pi/300)), int(B_joint1/(np.pi/300))]),
                          np.array([int(C_base1/(np.pi/300)), int(C_joint1/(np.pi/300))])])
    path1, path2 = plan.get_paths(waypoints, world)
    #plan.graph_path(path1, L1, L2)

    angles=path1+path2#[base_angle, joint_angle]
    angles=np.asarray(angles)
    numberOfWaypoints = len(angles) # Change this based on your path
    crash=np.delete(crash,0,0)
    plan.graph_path(angles, crash, L1, L2)


    pidJoint= Pid(.003,0,0.00009)
    pidBase = Pid(0.0045,0.0000015,-0.00025)

    arm.reset() # start simulation

    for waypoint in range(numberOfWaypoints):

        # Get current waypoint
        wbase_angle=angles[waypoint][0]*(np.pi/300)
        wjoint_angle=angles[waypoint][1]*(np.pi/300)

        for timeStep in range(stepsForEachMove):

            tic = time.perf_counter() # timer to maintain loop frequency

            BaseError = anglediff(wbase_angle,arm.state[0])# Step 1.1: Calculate  error based on light sensors
            dBaseError =sign(BaseError)*arm.state[2]# Step 1.2: Calculate change in error based on light sensors
            iBaseError = pidBase.cummulativeError-dBaseError*timeStep# Step 1.3: Calculate cummulative error based on light sensors

            pidBase.error = BaseError # set the class attributes
            pidBase.changeInError = dBaseError # set the class attributes
            pidBase.cummulativeError = iBaseError # set the class attributes

            JointError = anglediff(wjoint_angle,arm.state[1])# Step 1.1: Calculate  error based on light sensors
            dJointError = sign(JointError)*arm.state[3]# Step 1.2: Calculate change in error based on light sensors

            pidJoint.error = JointError # set the class attributes
            pidJoint.changeInError = dJointError # set the class attributes
            # Control arm to reach this waypoint

            actionHere1 = pidBase.action(timeStep) # Nm torque # Change this based on your controller
            actionHere2 = pidJoint.action(timeStep) # Nm torque # Change this based on your controller

            pidBase.previousError = BaseError # Set previous error to current error
            pidJoint.previousError = JointError # Set previous error to current error
            length=3.75*2.54/100#m
            g=9.81#m/s**2
            m=0.005#kg
            feed1=m*g*np.cos(arm.state[0])*length/2
            #feed2=0.001*np.cos(arm.state[0]+arm.state[1])

            arm.render() # Update rendering
            state, reward, terminal , __ = arm.step(actionHere1+feed1, actionHere2)
            time.sleep(10)

    print("Done")
    input("Press Enter to close...")
    arm.close()
