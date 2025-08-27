import serial
import time
import numpy as np
import os
import matplotlib.pyplot.animate as anim

#this code reads the serial line from the sunsensor and plots its sunvector and other info

PORT = "COM8"  # Replace with your port
BAUDRATE = 57600

global latest_values  # x, y, z, theta, phi
latest_values = [-1,-1,-1,-1,-1] # vector info
ser = serial.Serial(PORT, BAUDRATE, timeout=1)


fig = plt.figure(figsize=(9, 5))
ax_3d = fig.add_subplot(121, projection='3d')
ax_txt = fig.add_subplot(122)
ax_txt.axis('off')
ax_3d.clear()
ax_3d.set_xlim([-1, 1])
ax_3d.set_ylim([-1, 1])
ax_3d.set_zlim([-1, 1])
ax_3d.set_xlabel("X")
ax_3d.set_ylabel("Y")
ax_3d.set_zlabel("Z")

def serialRead():
    try:
        # Reads serial info
        line = ser.readline().decode('utf-8').strip()
        if not line:
            return
        parts = line.split(',')
        if len(parts) != 5:
            return
        # Turn it to list
        values = list(map(float, parts))
        global latest_values
        latest_values = values
    except Exception:
        print("not as expected")

def update(frame):
    serialRead()
    x, y, z, theta, phi = latest_values

    # Update 3D vector
    if x > -0 & y > -0 > z> -0:
        ax_3d.quiver(0, 0, 0, x, y, z, length=1.0, normalize=True, color='tab:red')
    else:
        ax_3d.quiver(0, 0, 0, x, y, z, length=1.0, normalize=True, color='tab:red')

    # Update side text
    ax_txt.clear()
    ax_txt.axis('off')
    ax_txt.text(0.1, 0.8, f"X= {x:.2f}")
    ax_txt.text(0.1, 0.6, f"Y= {y:.2f}")
    ax_txt.text(0.1, 0.4, f"Z= {z:.2f}")
    ax_txt.text(0.1, 0.2, f"$\\theta$= {theta:.2f} $\\degree$")
    ax_txt.text(0.1, 0.0, f"$\phi$= {phi:.2f} $\\degree$")

#Run animation (stops when window is closed)
ani = animation.FuncAnimation(fig, update, interval=200)  #Every 200 ms
plt.show()

# quit when window is closed
ser.close()