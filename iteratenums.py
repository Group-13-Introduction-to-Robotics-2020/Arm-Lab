import numpy as np
import time
import matplotlib.pyplot as plt

mm=25.4 #conversion inches to mm later... not yet
lower_length=3.75#*mm
upper_length=2.5#*mm

lower_theta=np.linspace(0,np.pi,300)#rads
upper_theta=np.linspace(-np.pi,np.pi,600)#rads

#variable label: object_number=[lowerx,lowery,upperx,uppery]
ob1=np.array([-4,4,-2,6])
ob2p1=np.array([-1,1,0,3])
ob2p2=np.array([0,2,1,3])
ob3=np.array([1,5,3,7])
crash=np.array([[0,0,0,0]])
#count=0
for lower_angle in lower_theta:
    for upper_angle in upper_theta:
        end_angle=lower_angle+upper_angle
        x=lower_length*np.cos(lower_angle)+upper_length*np.cos(end_angle)
        y=lower_length*np.cos(lower_angle)+upper_length*np.sin(end_angle)
        
        if (x>=ob1[0] and x<=ob1[2] and y>=ob1[1] and y<=ob1[3] or
        x>=ob2p1[0] and x<=ob2p1[2] and y>=ob2p1[1] and y<=ob2p1[3] or
        x>=ob2p2[0] and x<=ob2p2[2] and y>=ob2p2[1] and y<=ob2p2[3] or
        x>=ob3[0] and x<=ob3[2] and y>=ob3[1] and y<=ob3[3]):
            crash=np.append(crash,np.array([[180/np.pi*(lower_angle),180/np.pi*(upper_angle),x,y]]),axis=0)
crash=np.delete(crash,0,0)
print(crash)
plt.plot(crash[:,0],crash[:,1],'.')
plt.title("Configuration Space of Two Link Robot")
plt.ylabel("Angle between first and second link in degrees")
plt.xlabel("Angle at the base in degrees")
plt.axis([0,360,-180,180])
plt.show()


