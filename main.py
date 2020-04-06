import time
from arm import AcrobotEnv
import sys
import math
import copy
import numpy as np

boundaryLines = [
    (np.array([1, 7]), np.array([3, 7])),
    (np.array([1, 5]), np.array([1, 7])),
    (np.array([3, 5]), np.array([3, 7])),
    (np.array([1, 5]), np.array([3, 5])),
    (np.array([-4, 6]), np.array([-2, 6])),
    (np.array([-4, 4]), np.array([-4, 6])),
    (np.array([-2, 4]), np.array([-2, 6])),
    (np.array([-4, 4]), np.array([-2, 4])),
    (np.array([-1, 1]), np.array([0, 1])),
    (np.array([0, 2]), np.array([0, 1])),
    (np.array([0, 2]), np.array([1, 2])),
    (np.array([1, 3]), np.array([1, 2])),
    (np.array([1, 3]), np.array([-1, 3])),
    (np.array([-1, 1]), np.array([-1, 3])),

]

def intersect(l1, l2):
    l1a, l2a = l1
    l1b, l2b = l2

    da = l2a - l1a
    db = l2b - l1b
    dp = l1a - l1b

    dap = np.array([0.0, 0.0])
    dap[0] = -da[1]
    dap[1] = da[0]

    dot = dap[0]*db[0] + dap[1]*db[1]
    dot2 = dap[0]*dp[0] + dap[1]*dp[1]

    inter = (dot2/dot)*db + l1b

    upperXL1 =  inter[0] <= max(l1a[0], l2a[0])
    upperYL1 =  inter[1] <= max(l1a[1], l2a[1])
    upperXL2 = inter[0] <= max(l1b[0], l2b[0])
    upperYL2 = inter[1] <= max(l1b[1], l2b[1])
    lowerXL1 = inter[0] >= min(l1a[0], l2a[0])
    lowerYL1 = inter[1] >= min(l1a[1], l2a[1])
    lowerXL2 = inter[0] >= min(l1b[0], l2b[0])
    lowerYL2 = inter[1] >= min(l1b[1], l2b[1])

    return upperXL1 and upperXL2 and upperYL1 and upperYL2 and lowerXL1 and lowerXL2 and lowerYL1 and lowerYL2





def hasCollision(armSeg, boundaryLines):
    for line in boundaryLines:
        if intersect(armSeg, line):
            return True
    return False

def computeArmPos(j1, j2):
    firstPoint = np.array([3.75*np.cos(j1), 3.75*np.sin(j1)])
    secondPoint = np.array([firstPoint[0]+2.5*np.cos(j1+j2), firstPoint[1]+2.5*np.sin(j1+j2)])
    return (firstPoint, secondPoint)

def computeConfigSpace():
    validPoints = []
    for j1 in range(0,180):
        for j2 in range(-180, 180):
            pos = computeArmPos(j1*np.pi/180, j2*np.pi/180)
            if not hasCollision(pos, boundaryLines):
                validPoints.append([j1, j2])
    return validPoints


if __name__ == '__main__':

    arm = AcrobotEnv() # set up an instance of the arm class
    
    timeStep = 0.02 # sec
    timeForEachMove = 1 # sec
    stepsForEachMove = round(timeForEachMove/timeStep)

    # Make configuration space
    # Insert you code or calls to functions here
    pos = computeArmPos(np.pi/2, np.pi)
    check =(np.array([-1,2]), np.array([1,2]))

    valid = intersect(pos, check)
    print("pi/2, -pi/4 should be at: {} and is {}".format(pos, valid))

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

    # Plan a path
    # Insert your code or calls to functions here
    numberOfWaypoints = 10 # Change this based on your path
    
    arm.reset() # start simulation
    
    for waypoint in range(numberOfWaypoints):

        # Get current waypoint

        for timeStep in range(stepsForEachMove):

            tic = time.perf_counter()

            # Control arm to reach this waypoint

            actionHere1 = 0 # N torque # Change this based on your controller
            actionHere2 = 0 # N torque # Change this based on your controller
            
            arm.render() # Update rendering
            state, reward, terminal , __ = arm.step(actionHere1, actionHere2)
        
    print("Done")
    input("Press Enter to close...")
    arm.close()
