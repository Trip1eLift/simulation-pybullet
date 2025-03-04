import pybullet as p
import pybullet_data
import time
import numpy as np
from drone import Drone  # Import the Drone class

TimeStep = 1./120.

# Initialize PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(TimeStep)

# Create drone
drone1 = Drone("models/drone.urdf", start_pos=[0, 0, 1])
p.changeDynamics(drone1.drone_id, -1, linearDamping=0, angularDamping=0)

# Simulation loop
sim_time = 0
while True:
    drone1.update_sensors()
    
    # Get sensor data
    imu_data = drone1.get_imu_data()
    gps_data = drone1.get_gps_data()
    baro_data = drone1.get_barometer_data()
    mag_data = drone1.get_magnetometer_data()
    
    # Debug text
    time_text = f"Time: {sim_time:.2f} s"
    gps_text = f"GPS: [{gps_data['position'][0]:.2f}, {gps_data['position'][1]:.2f}, {gps_data['position'][2]:.2f}] m"
    baro_text = f"Baro Alt: {baro_data['altitude']:.2f} m"
    mag_text = f"Heading: {np.degrees(mag_data['heading']):.1f}°"
    imu_acc_text = f"Accel: [{imu_data['acceleration'][0]:.2f}, {imu_data['acceleration'][1]:.2f}, {imu_data['acceleration'][2]:.2f}] m/s²"
    imu_gyro_text = f"Gyro: [{imu_data['angular_velocity'][0]:.2f}, {imu_data['angular_velocity'][1]:.2f}, {imu_data['angular_velocity'][2]:.2f}] rad/s"

    # Display
    p.addUserDebugText(time_text, [0, 0, 1.6], textSize=1, lifeTime=0.1)
    
    # Apply thrust
    drone1.apply_motor_forces([10, 10, 10, 10])

    p.stepSimulation()
    #time.sleep(TimeStep)
    sim_time += TimeStep