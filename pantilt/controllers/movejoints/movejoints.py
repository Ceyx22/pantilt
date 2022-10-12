"""movejoints controller."""


# Step 1: Import the modules.  We'll use 'Robot' to access/control the
# robot and 'numpy' if we want to do any fancy math.
from controller import Robot
import numpy as np


# Step 2: # Define the devices...
# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Extract the two motors.
panmotor  = robot.getDevice('pan')
tiltmotor = robot.getDevice('tilt')

# Also enable the camera (so you can see the video feed).
robot.getDevice("camera").enable(timestep)


# Step 3: Any initialization you want to do for your code?
Apan  = 1.0             # Amplitude for pan joint
Atilt = 0.5             # Amplitude for tilt joint


# Step 4: Main continuous loop
while True:
    # Take one time step in the simulation.  At the end of this, time
    # will have advanced by (timestep).  Should the simulation run
    # into a problem (for example if you reset the simulation), this
    # will return (-1) and we should break out of the loop.
    if robot.step(timestep) == -1:
        break

    # Grab the current time.
    t = robot.getTime()

    # Set the pan and tilt angles.  You will DEFINITELY want update these.
    pan  = Apan  * (1 - np.cos(t))
    tilt = Atilt * np.sin(0.5 * t)
    
    # Send the commands.
    panmotor.setPosition(pan)
    tiltmotor.setPosition(tilt)

    # Any bookkeeping you want to do?  Include the print statement to
    # graph the trajectories.
    print(t, pan, tilt)

    # You could also break the loop at the "end of time"
    if t>=15.0:
        break


# Step 5: Clean up the code.  Nothing to do in this example.
pass
