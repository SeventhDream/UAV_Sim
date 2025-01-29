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
uavPath = os.path.abspath("cf2x.urdf")
startPos = [0, 0, 2] # UAV starts at (0,0) at 2m height.
startOrientation = p.getQuaternionFromEuler([0, 0, 0]) # No rotation.

# Load UAV model.
uavId = p.loadURDF(uavPath, startPos, startOrientation) # Temporary UAV model.

# Keyboard Input.
def get_keyboard_control():
    keys = p.getKeyboardEvents()
    thrust = 0

    # Control thrusts with up and down arrow keys.
    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW]:
        thrust = 5
    elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW]:
        thrust = -5

    return thrust

# Simulation loop.
while True:
    thrust = get_keyboard_control()

    # Apply force (simulating rotor thrust).
    p.applyExternalForce(uavId, -1, [0, 0, thrust], [0, 0, 0], p.WORLD_FRAME)

    p.stepSimulation()
    time.sleep(1/100.0) # Run at 100 Hz.