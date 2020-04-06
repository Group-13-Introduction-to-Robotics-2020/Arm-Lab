import time
from arm import AcrobotEnv
import sys
import math
import copy
import numpy as np


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

    observation = arm.reset() #not sure if needed Remember:
    #observation is [left reading, right reading, encoder value]

    timeStep = 0.02 # sec
    timeForEachMove = 10 # sec
    stepsForEachMove = round(timeForEachMove/timeStep)

    # Make configuraton space
    # Insert you code or calls to functions here

    # Get three waypoints from the user
#    Ax = int(input("Type Ax: "))
#    Ay = int(input("Type Ay: "))
#    Bx = int(input("Type Bx: "))
#    By = int(input("Type By: "))
#    Cx = int(input("Type Cx: "))
#    Cy = int(input("Type Cy: "))
    Ax=5
    Ay=3
    Bx=0
    By=5
    Cx=-3
    Cy=3

    arm.Ax = Ax*0.0254; # Simulaiton is in SI units
    arm.Ay = Ay*0.0254; # Simulaiton is in SI units
    arm.Bx = Bx*0.0254; # Simulaiton is in SI units
    arm.By = By*0.0254; # Simulaiton is in SI units
    arm.Cx = Cx*0.0254; # Simulaiton is in SI units
    arm.Cy = Cy*0.0254; # Simulaiton is in SI units

    # Inverse Kinematics
    # 6 sets of angles, 2 sets for each coordinate (A,B,C) for the two solutions for that point
    # (A_base1, A_joint1), (A_base2, A_joint2), (B_base1, B_joint1), ...
    L1 = 3.75 #inches
    L2 = 2.5 #inches

    A_joint1 = np.arccos((arm.Ax^2 + arm.Ay^2 - L1^2 - L2^2)/(2*L1*L2))
    A_base1 = np.arctan2(arm.Ay, arm.Ax) - np.arcsin((L2*np.sin(A_joint1))/np.sqrt(arm.Ax^2 + arm.Ay^2))
    A_joint2 = (2*np.pi) - A_joint1
    A_base2 = np.arctan2(arm.Ay, arm.Ax) - np.arcsin((L2*np.sin(A_joint2))/np.sqrt(arm.Ax^2 + arm.Ay^2))

    B_joint1 = np.arccos((arm.Bx^2 + arm.By^2 - L1^2 - L2^2)/(2*L1*L2))
    B_base1 = np.arctan2(arm.By, arm.Bx) - np.arcsin((L2*np.sin(B_joint1))/np.sqrt(arm.Bx^2 + arm.By^2))
    B_joint2 = (2*np.pi) - B_joint1
    B_base2 = np.arctan2(arm.By, arm.Bx) - np.arcsin((L2*np.sin(B_joint2))/np.sqrt(arm.Bx^2 + arm.By^2))

    C_joint1 = np.arccos((arm.Cx^2 + arm.Cy^2 - L1^2 - L2^2)/(2*L1*L2))
    C_base1 = np.arctan2(arm.Cy, arm.Cx) - np.arcsin((L2*np.sin(C_joint1))/np.sqrt(arm.Cx^2 + arm.Cy^2))
    C_joint2 = (2*np.pi) - C_joint1
    C_base2 = np.arctan2(arm.Cy, arm.Cx) - np.arcsin((L2*np.sin(C_joint2))/np.sqrt(arm.Cx^2 + arm.Cy^2))


    # Plan a path
    # Insert your code or calls to functions here

    angles=np.array([[50,40],[55,44],[60,50]])#[base_angle, joint_angle]
    angles=angles/180*np.pi
    numberOfWaypoints = len(angles) # Change this based on your path
    print(numberOfWaypoints)#should display 3 at the moment
    
    pidJoint= Pid(.003,0,-.00009)#
    pidBase = Pid(0.0045,0,0.0002)
    arm.reset() # start simulation

    for waypoint in range(numberOfWaypoints):

        # Get current waypoint
        wbase_angle=angles[waypoint,0]
        wjoint_angle=angles[waypoint,1]

        for timeStep in range(stepsForEachMove):

            tic = time.perf_counter() # timer to maintain loop frequency

            BaseError = anglediff(wbase_angle,arm.state[0])# Step 1.1: Calculate  error based on light sensors
            dBaseError = -1*sign(BaseError)*arm.state[2]# Step 1.2: Calculate change in error based on light sensors
            iBaseError = pidBase.cummulativeError+dBaseError*timeStep# Step 1.3: Calculate cummulative error based on light sensors

            pidBase.error = BaseError # set the class attributes
            pidBase.changeInError = dBaseError # set the class attributes
            pidBase.cummulativeError = iBaseError # set the class attributes

            JointError = anglediff(wjoint_angle,arm.state[1])# Step 1.1: Calculate  error based on light sensors
            dJointError = -1*sign(JointError)*arm.state[3]# Step 1.2: Calculate change in error based on light sensors
            iJointError = pidJoint.cummulativeError+dJointError*timeStep# Step 1.3: Calculate cummulative error based on light sensors

            pidJoint.error = JointError # set the class attributes
            pidJoint.changeInError = dJointError # set the class attributes
            pidJoint.cummulativeError = iJointError # set the class attributes
            # Control arm to reach this waypoint

            actionHere1 = pidBase.action(timeStep) # Nm torque # Change this based on your controller
            actionHere2 = pidJoint.action(timeStep) # Nm torque # Change this based on your controller
            #print("The Joint error is",JointError,"The Base error is",BaseError)
            #print("torque Joint is %g" %actionHere2)
            #print("torque Base is %g" %actionHere1)
            
            pidBase.previousError = BaseError # Set previous error to current error
            pidJoint.previousError = JointError # Set previous error to current error
            length=3.75*2.54/100#m
            g=9.81#m/s^2
            m=0.005#kg
            feed=m*g*np.cos(arm.state[0])*length/2
            #print(arm.state[0])

            arm.render() # Update rendering
            state, reward, terminal , __ = arm.step(actionHere1+feed, actionHere2)
        
    print("Done")
    input("Press Enter to close...")
    arm.close()


