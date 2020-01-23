# A script to display the results of the last execution of MagnetLoc.
# MagnetLoc stores its results in 'log.txt' and the meaningful input data
# in inputLog.mat (among which robot parameters and noise variances).
# Graphs displayed:
# Figure 1:
#   - The path calculated by odometry only (red).
#   - The path estimated by the Kalman filter (blue).
#   - The real path (green).
#   - The locations of the magnets which have been detected (black dots).
#   - The estimated locations of the detected line in absolute frame
#       using the measurement (that's variable oMeasMagnet in the program).
# Figure 2:
#   - Speed and rotation speed, as estimated using the encoders.
# Figure 3:
#   - Estimated error standard deviations (extracted directly from
#       the diagonal of P, hence in absolute frame.
# Figure 4:
#   - Estimated error standard deviations in robot frame.
# Figure 5:
#   - Mahalanobis distances calculated with the line closest to
#      the measurement point (candidate line) in blue.
#   - Mahalanobis distances calculated with the four nearest neighbor
#       lines of candidate line (2 with the same orientatin and 2 with a different orientation )in red.
#   - Mahalanobis distance threshold used in the program (black line).
# Figure 6:
#   - Estimated x, y, theta as functions of time.
# Figure 7:
#   - Number of sensor on a line at each time instant.
# Figure 8:
#   - Raw sensor measurements as a function of the curvilinear abscissa
#       (distance traveled by point M). The vertical axis represents the
#       state of each sensor. A vertical line indicates a closed
#       sensor (a sensor which detects a line).
#   - You may comment out this graph when you don't need it anymore
#       (when you're done estimating the measurement noise).

import matplotlib.pyplot as plt
import math

# Valeurs random
x=[1,2,3,4,5,6]
y=[6,5,4,3,2,1]
theta=[-math.pi/4, -math.pi/4, -math.pi/4, -math.pi/4, -math.pi/4, -math.pi/4]
treal=[1,2,3,4,5,6]
nbSensors = 2
sensorState=[[0,1,0,0,1,1],[0,0,1,0,0,1]]
tOdo = treal
U=[[1,1,1,1,1,1],[0,0,0,0,0,0]]
samplingPeriod = 0.01

# Figure test
plt.figure(1)  
plt.plot([1, 2, 3, 4])
plt.ylabel('some numbers')
plt.show()

# Figure 1

# Figure 2
plt.figure(2)
plt.subplot(211)
U1 = [x*1/samplingPeriod for x in U[0]]
plt.plot( tOdo, U1)
plt.xlabel('t (s)')
plt.ylabel('v (mm/s)')
plt.title('Odometry-estimated speed')
plt.subplot(2,1,2)
U2 = [x*180/math.pi/samplingPeriod for x in U[1]]
plt.plot( tOdo,U2)
plt.xlabel('t (s)')
plt.ylabel('w (deg/s)' )
plt.title('Odometry-estimated rotation speed')

#...

# Figure 6
plt.figure(3)  
plt.subplot(311)
plt.plot(x)
plt.ylabel('x (m)')
plt.xlabel('t (s)')
plt.subplot(312)
plt.plot(y)
plt.ylabel('y (m)')
plt.xlabel('t (s)')
plt.subplot(313)
plt.plot(theta)
plt.ylabel('theta (rad)')
plt.xlabel('t (s)')
plt.show()

# Figure 7
plt.figure(4)
nbMeas=[]
for time in range(len(treal) ):
    Mesures=0
    for sensor in range(nbSensors):
        Mesures=Mesures+sensorState[sensor][time]
    nbMeas.append(Mesures)

plt.plot(nbMeas)
plt.ylabel('nbMeas')
plt.xlabel('t (s)')
plt.show()






