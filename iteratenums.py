import numpy as np
import time
import matplotlib.pyplot as plt

def getConfigSpace():
    lower_length=3.75
    upper_length=2.5#+.5/2.54#incresed the length of the arm by .5in
    lower_theta=np.linspace(0,np.pi,300)#rads
    upper_theta=np.linspace(-np.pi,np.pi,600)#rads

    crash_grid = np.zeros((300,600))

    #variable label: object_number=[lowerx,lowery,upperx,uppery
    r=0#in radius of bot to bloat obstacles
    ob1=np.array([-4-r,4-r,-2+r,6+r])
    ob2p1=np.array([-1-r,1-r,0+r,3+r])
    ob2p2=np.array([0-r,2-r,1+r,3+r])
    ob3=np.array([1-r,5-r,3+r,7+r])
    crash=np.array([[0,0,0,0]])
    #count=0
    i = 0
    j = 0
    for lower_angle in lower_theta:
        j = 0
        for upper_angle in upper_theta:
            end_angle=lower_angle+upper_angle
            x=lower_length*np.cos(lower_angle)+upper_length*np.cos(end_angle)
            y=lower_length*np.sin(lower_angle)+upper_length*np.sin(end_angle)

            if (x>=ob1[0] and x<=ob1[2] and y>=ob1[1] and y<=ob1[3] or
            x>=ob2p1[0] and x<=ob2p1[2] and y>=ob2p1[1] and y<=ob2p1[3] or
            x>=ob2p2[0] and x<=ob2p2[2] and y>=ob2p2[1] and y<=ob2p2[3] or
            x>=ob3[0] and x<=ob3[2] and y>=ob3[1] and y<=ob3[3]):
                crash=np.append(crash,np.array([[180/np.pi*(lower_angle),180/np.pi*(upper_angle),x,y]]),axis=0)
                crash_grid[i][j] = 1
            j += 1
        i += 1
    return crash_grid,crash

"""
crash=np.delete(crash,0,0)
plt.plot(crash[:,0],crash[:,1],'.')
plt.title("Configuration Space of Two Link Robot")
plt.ylabel("Angle between first and second link in degrees")
plt.xlabel("Angle at the base in degrees")
plt.axis([0,180,-180,180])
plt.show()
"""


