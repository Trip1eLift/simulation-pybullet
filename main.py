import trimesh
import pybullet as p
import pybullet_data
import time

def convert_glb(file_name):
    # Load the .glb file
    mesh = trimesh.load(f"{file_name}.glb")
    # Export as .obj
    mesh.export(f"{file_name}.obj")

#convert_glb("apollo_lunar_module")

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To find URDFs
drone_id = p.loadURDF("drone.urdf", basePosition=[0, 0, 2])

# Indices of the motors (these depend on the URDF structure)
motor_positions = [
    [0.2, 0.2, 0],   # Motor 1 position
    [-0.2, 0.2, 0],  # Motor 2 position
    [-0.2, -0.2, 0], # Motor 3 position
    [0.2, -0.2, 0]   # Motor 4 position
]

# Run the simulation
while True:
    # Apply an **upward thrust** to each motor
    force_value = 9.8  # Newtons
    for pos in motor_positions:
        p.applyExternalForce(
            objectUniqueId=drone_id,  
            linkIndex=-1,  # -1 means apply force to the whole body, not a specific joint
            forceObj=[0, 0, force_value],  # Apply force in the Z (upward) direction
            posObj=pos,  # Apply at motor position
            flags=p.WORLD_FRAME
        )

    p.stepSimulation()
    time.sleep(1./240.)