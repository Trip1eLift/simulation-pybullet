import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from pid import PIDController

class Drone:
    def __init__(self, urdf_path, start_pos=[0, 0, 1], dt=1./120.):
        self.drone_id = p.loadURDF(urdf_path, basePosition=start_pos)
        
        self.motor_positions = [
            [0.16, 0.13, 0.05],   # Front-Left
            [0.16, -0.13, 0.05],  # Front-Right
            [-0.16, 0.13, 0.05],  # Back-Left
            [-0.16, -0.13, 0.05]  # Back-Right
        ]
        
        # Sensor data
        self.gps_position = np.array(start_pos)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.orientation_euler = np.array([0.0, 0.0, 0.0])
        self.altitude = start_pos[2]
        self.heading = 0.0
        
        # Sensor noise
        self.gps_noise_std = 0.02
        self.accel_noise_std = 0.01
        self.gyro_noise_std = 0.005
        self.baro_noise_std = 0.05
        self.mag_noise_std = 0.02

        # PID controllers
        self.roll_pid = PIDController(kp=1.0, ki=0.02, kd=0.05, dt=dt)
        self.pitch_pid = PIDController(kp=1.0, ki=0.02, kd=0.05, dt=dt)
        self.altitude_pid = PIDController(kp=0.1, ki=0.0, kd=0.3, dt=dt)

        # Motor dynamics
        self.max_spin_speed = 4189.0  # rad/s (~40,000 RPM)
        self.max_torque = 0.1  # Nm
        self.current_spin_speeds = [495.22, 495.22, 495.22, 495.22]  # Hover speed for 1kg
        self.k_thrust = 0.00001  # Thrust coefficient
        self.dt = dt

    def update_sensors(self):
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone_id)
        rot = R.from_quat(orn)
        
        world_accel = np.array([0, 0, -9.81])
        body_accel = rot.inv().apply(world_accel)
        self.acceleration = body_accel + np.random.normal(0, self.accel_noise_std, 3)
        self.angular_velocity = np.array(ang_vel) + np.random.normal(0, self.gyro_noise_std, 3)
        self.gps_position = np.array(pos) + np.random.normal(0, self.gps_noise_std, 3)
        self.velocity = np.array(vel) + np.random.normal(0, self.gps_noise_std/0.1, 3)
        true_altitude = pos[2]
        self.altitude = true_altitude + np.random.normal(0, self.baro_noise_std)
        euler = rot.as_euler('xyz')
        self.heading = euler[2] + np.random.normal(0, self.mag_noise_std)
        self.orientation_euler = euler

    def get_imu_data(self):
        return {'acceleration': self.acceleration.tolist(), 'angular_velocity': self.angular_velocity.tolist()}

    def get_gps_data(self):
        return {'position': self.gps_position.tolist(), 'velocity': self.velocity.tolist()}

    def get_barometer_data(self):
        return {'altitude': self.altitude}

    def get_magnetometer_data(self):
        return {'heading': self.heading}
    
    def get_motor_speeds(self):
        return self.current_spin_speeds

    def apply_motor_forces(self, forces):
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        rot_matrix = R.from_quat(orn).as_matrix()
        for pos_local, force in zip(self.motor_positions, forces):
            pos_world = np.dot(rot_matrix, pos_local) + np.array(pos)
            # Force direction remains upward in body frame, magnitude and sign from thrust
            force_world = np.dot(rot_matrix, [0, 0, force])  # Negative force goes downward
            p.applyExternalForce(
                objectUniqueId=self.drone_id,
                linkIndex=-1,
                forceObj=force_world.tolist(),
                posObj=pos_world.tolist(),
                flags=p.WORLD_FRAME
            )

    def update_spin_speeds(self, target_speeds):
        """Update motor spin speeds with torque constraints and apply forces"""
        forces = [0.0] * 4
        for i in range(4):
            delta_omega = target_speeds[i] - self.current_spin_speeds[i]
            max_alpha = self.max_torque / 0.001  # Inertia ~0.001 kg·m²
            max_delta_omega = max_alpha * self.dt
            delta_omega = np.clip(delta_omega, -max_delta_omega, max_delta_omega)
            self.current_spin_speeds[i] += delta_omega
            # Allow negative spin speeds for downward thrust
            self.current_spin_speeds[i] = np.clip(self.current_spin_speeds[i], -self.max_spin_speed, self.max_spin_speed)
            # Thrust = k_thrust * omega^2 * sign(omega) to preserve direction
            forces[i] = self.k_thrust * (self.current_spin_speeds[i] ** 2) * np.sign(self.current_spin_speeds[i])
        
        self.apply_motor_forces(forces)
        return self.current_spin_speeds

    def balance(self, target_altitude=1.0):
        """Compute target spin speeds to stabilize roll, pitch, and altitude"""
        self.update_sensors()
        
        target_roll = 0.0
        target_pitch = 0.0
        
        roll = self.orientation_euler[0]
        pitch = self.orientation_euler[1]
        altitude = self.altitude
        
        # Reduced scaling factor from 100 to 10
        roll_output = self.roll_pid.compute(target_roll, roll) * 100.0
        pitch_output = self.pitch_pid.compute(target_pitch, pitch) * 100.0
        altitude_output = self.altitude_pid.compute(target_altitude, altitude) * 100.0
        
        base_spin = np.sqrt(9.81 / (4 * self.k_thrust))
        
        target_speeds = [
            base_spin + altitude_output - roll_output + pitch_output,  # FL
            base_spin + altitude_output + roll_output + pitch_output,  # FR
            base_spin + altitude_output - roll_output - pitch_output,  # BL
            base_spin + altitude_output + roll_output - pitch_output   # BR
        ]
        
        return self.update_spin_speeds(target_speeds)