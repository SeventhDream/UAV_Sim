import pybullet as p
import pybullet_data
import time
import os

# Initialise physics simulation.
physicsClient = p.connect(p.GUI) # GUI mode for visualisation.
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Load built-in assets.
p.setGravity(0, 0, -9.81) # Earth's gravity.

# Load a simple plane as ground.
planeId = p.loadURDF("plane.urdf")

# Initialise UAV.
uav_path = os.path.abspath("cf2x.urdf")
start_pos = [0, 0, 2] # UAV starts at (0,0) at 2m height.
start_orientation = p.getQuaternionFromEuler([0, 0, 0]) # No rotation.

# Load UAV model.
uavId = p.loadURDF(uav_path, start_pos, start_orientation) # Temporary UAV model.

# Simulation loop.
for _ in range(1000):
    p.stepSimulation() # Run physics calculations.
    time.sleep(1 / 240.0) # Simulate at 240 Hz

p.disconnect()