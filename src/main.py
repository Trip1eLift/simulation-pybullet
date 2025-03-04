import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
from drone import Drone  # Import the Drone class
from pid import PIDController

TimeStep = 1./30.

# Initialize PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(TimeStep)

# Create drone
drone1 = Drone("models/drone.urdf", start_pos=[0, 0, 0], dt=TimeStep)
p.changeDynamics(drone1.drone_id, -1, linearDamping=0, angularDamping=0)

# Prepare plotting
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))

# Data storage for plotting
time_data = []
height_data = []
velocity_data = []
motor_speed_data = []

# Create sliders for PID constants and target altitude
kp_slider = p.addUserDebugParameter("Altitude PID - Kp", 0.0, 0.5, 0.1)
ki_slider = p.addUserDebugParameter("Altitude PID - Ki", 0.0, 0.5, 0.0)
kd_slider = p.addUserDebugParameter("Altitude PID - Kd", 0.0, 0.5, 0.3)
slider_id = p.addUserDebugParameter("Target Altitude", 0.0, 10.0, 3.0)  # Set initial value to 3.0 meters

time_text_id = p.addUserDebugText("Time: 0.00 s", [0, 0, 1.6], textSize=1, lifeTime=0.5)

# Simulation loop
sim_time = 0
while True:
    # Display debug information
    time_text = f"Time: {sim_time:.2f} s"
    p.addUserDebugText(time_text, [0, 0, 1.6], textSize=1, lifeTime=0.5)

    try:
        # Get the current value of the target altitude from the slider
        target_altitude = p.readUserDebugParameter(slider_id)

        # Read PID values from sliders
        kp = p.readUserDebugParameter(kp_slider)
        ki = p.readUserDebugParameter(ki_slider)
        kd = p.readUserDebugParameter(kd_slider)
        # Print the PID values and target altitude
        print(f"Target Altitude: {target_altitude}, PID values: Kp={kp}, Ki={ki}, Kd={kd}")
        
        # Update the altitude PID controller with the new values
        drone1.altitude_pid.update_constants(kp, ki, kd)

        # Apply thrust (use the dynamic target_altitude from the slider)
        forces = drone1.balance(target_altitude=target_altitude)

    except Exception as e:
        print(f"Error reading parameters: {e}")

    # Get sensor data
    imu_data = drone1.get_imu_data()
    gps_data = drone1.get_gps_data()
    baro_data = drone1.get_barometer_data()
    mag_data = drone1.get_magnetometer_data()

    # Get height (z position)
    height = gps_data['position'][2]
    
    # Get velocity (linear velocity from the IMU or through PyBullet)
    velocity, angular_velocity = p.getBaseVelocity(drone1.drone_id)
    linear_velocity = np.linalg.norm(velocity)  # Magnitude of the velocity vector

    # Get motor speeds (assuming the Drone class has a method to get this)
    motor_speeds = drone1.get_motor_speeds()
    
    # Calculate average motor speed
    avg_motor_speed = np.mean(motor_speeds)

    # Store data for plotting
    time_data.append(sim_time)
    height_data.append(height)
    velocity_data.append(linear_velocity)
    motor_speed_data.append(avg_motor_speed)

    # Update the plots
    ax1.clear()
    ax2.clear()
    ax3.clear()

    ax1.plot(time_data, height_data, label="Height (m)")
    ax1.set_title("Height vs Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Height (m)")

    ax2.plot(time_data, velocity_data, label="Velocity (m/s)", color='orange')
    ax2.set_title("Velocity vs Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (m/s)")

    ax3.plot(time_data, motor_speed_data, label="Avg Motor Speed (RPM)", color='green')
    ax3.set_title("Average Motor Speed vs Time")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Motor Speed (RPM)")

    # Draw the updated plot
    plt.pause(0.01)

    # Step the simulation
    p.stepSimulation()
    sim_time += TimeStep
