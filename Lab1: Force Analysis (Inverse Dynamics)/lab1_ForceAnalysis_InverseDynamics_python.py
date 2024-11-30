# Force Analysis (Inverse Dynamics) of 1-Link robot

import math
import numpy as np
import matplotlib.pyplot as plt 

# Model Constants
m = 1
l = 1
lc = 0.5
Izz = (1/3)*m*l*l
g = 9.81

# Motion Parameter
d2r = math.pi / 180  # degree to radian
th0 = 0 * d2r        # initial position (angle)
thT = 120 * d2r      # final position (angle)
T = 3                # time taken
step = 0.01

# Initialize arrays
size = int(T / step) + 1
time = np.zeros(size)
thd = np.zeros(size)
dthd = np.zeros(size)
ddthd = np.zeros(size)
tau = np.zeros(size)
xe = np.zeros(size)
ye = np.zeros(size)
dxe = np.zeros(size)
dye = np.zeros(size)
ddxe = np.zeros(size)
ddye = np.zeros(size)

# Constraints for calculations
c1 = (thT - th0) / T
c2 = (2 * math.pi) / T

# For loop starts
for i, t in enumerate(np.arange(0, T + step, step)):
    time[i] = t
    thd[i] = th0 + c1 * (t - (1 / c2) * math.sin(c2 * t))  # Joint Position
    dthd[i] = c1 * (1 - math.cos(c2 * t))                 # Joint Velocity
    ddthd[i] = c1 * c2 * math.sin(c2 * t)                 # Joint Acceleration

    # Joint Torque
    tau[i] = Izz * ddthd[i] + m * g * lc * math.cos(thd[i])

    # End Effector Position
    xe[i] = l * math.cos(thd[i])
    ye[i] = l * math.sin(thd[i])

    # End Effector Velocity and Acceleration
    dxe[i] = -l * math.sin(thd[i]) * dthd[i]
    dye[i] = l * math.cos(thd[i]) * dthd[i]
    ddxe[i] = -l * math.cos(thd[i]) * dthd[i] - l * math.sin(thd[i]) * ddthd[i]
    ddye[i] = -l * math.sin(thd[i]) * dthd[i] + l * math.cos(thd[i]) * ddthd[i]

    # Animation
    xx = [0, xe[i]]
    yy = [0, ye[i]]
    plt.figure(2)
    plt.clf()
    plt.plot(xx, yy, label="Link")
    plt.plot(xe[:i+1], ye[:i+1], label="Trajectory")
    plt.axis([-2*l, 2*l, -2*l, 2*l])
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    # plt.axis("equal")
    plt.title("Trajectory of the 1-link robot")
    plt.pause(0.01)

# Final Plots
plt.figure(1)
plt.plot(time, thd / d2r, label="Joint angle (deg)")
plt.plot(time, dthd / d2r, label="Joint rate (deg/s)")
plt.plot(time, ddthd / d2r, label="Joint accln (deg/s²)")
plt.plot(time, tau, label="Torque (Nm)")
plt.xlabel('Time (s)')
plt.ylabel('Motion')
plt.title("Variables of the 1-link robot")
plt.legend()

plt.figure(3)
plt.plot(time, dxe, label="dxe (m/s)")
plt.plot(time, dye, label="dye (m/s)")
plt.plot(time, ddxe, label="ddxe (m/s²)")
plt.plot(time, ddye, label="ddye (m/s²)")
plt.xlabel('Time (s)')
plt.ylabel('End Effector Kinematics')
plt.title("Velocity and Accleration of endeffector of the 1-link robot")
plt.legend()

plt.show()

