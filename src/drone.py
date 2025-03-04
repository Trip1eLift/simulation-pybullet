import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R

class Drone:
    def __init__(self, urdf_path, start_pos=[0, 0, 1]):
        self.drone_id = p.loadURDF(urdf_path, basePosition=start_pos)
        
        # Motor positions relative to the drone's center
        self.motor_positions = [
            [0.16, 0.13, 0.05],   # Front-Left
            [0.16, -0.13, 0.05],  # Front-Right
            [-0.16, 0.13, 0.05],  # Back-Left
            [-0.16, -0.13, 0.05]  # Back-Right
        ]
        
        # Sensor data initialization
        self.gps_position = np.array(start_pos)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.orientation_euler = np.array([0.0, 0.0, 0.0])  # Roll, Pitch, Yaw
        self.altitude = start_pos[2]  # Barometer-based altitude
        self.heading = 0.0  # Magnetometer-based heading
        
        # Sensor noise parameters
        self.gps_noise_std = 0.02  # meters
        self.accel_noise_std = 0.01  # m/s²
        self.gyro_noise_std = 0.005  # rad/s
        self.baro_noise_std = 0.05  # meters
        self.mag_noise_std = 0.02  # radians

    def update_sensors(self):
        """Update all sensor readings with simulated noise"""
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone_id)
        rot = R.from_quat(orn)
        
        # IMU: Accelerometer (includes gravity)
        world_accel = np.array([0, 0, -9.81])  # Gravity in world frame
        body_accel = rot.inv().apply(world_accel)
        self.acceleration = body_accel + np.random.normal(0, self.accel_noise_std, 3)
        
        # IMU: Gyroscope
        self.angular_velocity = np.array(ang_vel) + np.random.normal(0, self.gyro_noise_std, 3)
        
        # GPS: Position
        self.gps_position = np.array(pos) + np.random.normal(0, self.gps_noise_std, 3)
        self.velocity = np.array(vel) + np.random.normal(0, self.gps_noise_std/0.1, 3)
        
        # Barometer: Altitude
        true_altitude = pos[2]
        self.altitude = true_altitude + np.random.normal(0, self.baro_noise_std)
        
        # Magnetometer: Heading
        euler = rot.as_euler('xyz')
        self.heading = euler[2] + np.random.normal(0, self.mag_noise_std)
        
        self.orientation_euler = euler

    def get_imu_data(self):
        """Return accelerometer and gyroscope data"""
        return {
            'acceleration': self.acceleration.tolist(),
            'angular_velocity': self.angular_velocity.tolist()
        }

    def get_gps_data(self):
        """Return position and velocity"""
        return {
            'position': self.gps_position.tolist(),
            'velocity': self.velocity.tolist()
        }

    def get_barometer_data(self):
        """Return altitude"""
        return {'altitude': self.altitude}

    def get_magnetometer_data(self):
        """Return heading"""
        return {'heading': self.heading}

    def apply_motor_forces(self, forces):
        """Apply force in the drone's local frame"""
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        rot_matrix = R.from_quat(orn).as_matrix()
        for pos_local, force in zip(self.motor_positions, forces):
            pos_world = np.dot(rot_matrix, pos_local) + np.array(pos)
            force_world = np.dot(rot_matrix, [0, 0, force])
            p.applyExternalForce(
                objectUniqueId=self.drone_id,
                linkIndex=-1,
                forceObj=force_world.tolist(),
                posObj=pos_world.tolist(),
                flags=p.WORLD_FRAME
            )