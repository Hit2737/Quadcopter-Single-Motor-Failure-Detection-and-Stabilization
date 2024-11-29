import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import SensorCombined, VehicleGlobalPosition, SensorGps
from std_msgs.msg import Float32MultiArray

import numpy as np

from utils import R_a2_theta, R_a1_phi, R_a3_psi, PIDController

class ThrottlePublisher(Node):
    def __init__(self):
        super().__init__('throttle_publisher')

        # Define QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # PID Controllers
        self.force_controller = PIDController(KP=np.array([10.0]), KD=np.array([6.0]), KI=np.array([0.0]), size=1)
        self.moment_controller = PIDController(KP=np.array([6.0, 6.0, 6.0]), KD=np.array([3.0, 3.0, 3.0]), KI=np.array([0.0, 0.0, 0.0]), size=3)
        self.attitude_controller = PIDController(KP=np.array([8.0, 8.0]), KD=np.array([4.0, 4.0]), KI=np.array([0.0, 0.0]), size=2)
        self.yaw_controller = PIDController(KP=np.array([5.0]), KD=np.array([2.0]), KI=np.array([0.0]), size=1)

        # Initialize data storage
        self.sensor_combined_data = None
        self.vehicle_gps_position_data = None
        self.vehicle_global_position_data = None

        # Create subscribers with QoS settings
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_combined_callback, qos_profile)
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_position_callback, qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)

        self.throttle_values = [0.0, 0.0, 0.0, 0.0]
        self.dt = 0.005
        self.publisher_ = self.create_publisher(Float32MultiArray, 'throttle_values', 10)
        self.control_timer = self.create_timer(self.dt, self.update_control_loop)
        self.throttle_timer = self.create_timer(self.dt, self.publish_throttle_values)

        # Desired_state = [X, Y, Z, Vx, Vy, Vz, Ax, Ay, Az, Phi, Theta, Psi]
        self.desired_state = [0.0, 0.0, 500.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.errors = [np.zeros(6)]

        self.mass = 1.5
        self.J = np.diag([0.029125, 0.029125, 0.055225])
        self.g = 9.81
        self.max_motor_speed = 1100

        # Motor Parameters
        FL_b1 = 0.13
        BL_b1 = 0.13
        FR_b1 = 0.13
        BR_b1 = 0.13

        FR_b2 = 0.22
        BL_b2 = 0.2
        BR_b2 = 0.2
        FL_b2 = 0.22

        KF = 5.84e-06
        KM = 0.06
        self.allocation_matrix = np.array([
            [KF, KF, KF, KF],
            [-KF * FR_b2, KF * BL_b2, KF * FL_b2, -KF * BR_b2],
            [-KF * FR_b1, KF * BL_b1, -KF * FL_b1, KF * BR_b1],
            [KM, KM, -KM, -KM]
        ])

    def publish_throttle_values(self):
        msg = Float32MultiArray()
        msg.data = self.throttle_values
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published throttle values: {msg.data}')

    def sensor_combined_callback(self, msg):
        self.sensor_combined_data = [msg.timestamp] + list(msg.gyro_rad) + list(msg.accelerometer_m_s2)

    def vehicle_gps_position_callback(self, msg):
        self.vehicle_gps_position_data = [msg.timestamp, msg.latitude_deg, msg.longitude_deg, msg.altitude_msl_m, msg.vel_m_s, msg.vel_n_m_s, msg.vel_e_m_s, msg.vel_d_m_s]
    
    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position_data = [msg.timestamp, msg.lat, msg.lon, msg.alt]

    def compute_body_frame_matrix(self, phi, theta, psi):
        return np.dot(R_a3_psi(psi), np.dot(R_a2_theta(theta), R_a1_phi(phi)))

    def compute_motor_speeds(self, F, M):
        omega_squared = np.linalg.inv(self.allocation_matrix) @ np.array([F, M[0], M[1], M[2]])
        self.omega = np.sqrt(np.clip(omega_squared, 0, self.max_motor_speed**2))
        self.omega = self.omega / self.max_motor_speed
        return self.omega
    
    def update_control_loop(self):
        current_state = [0] * 6

        current_state[:3] = self.vehicle_global_position_data[1:4]
        current_state[3:6] = self.sensor_combined_data[1:4]

        BFM = self.compute_body_frame_matrix(current_state[3], current_state[4], current_state[5])
        R_body = BFM[:3, :3]

        self.errors.append(np.zeros(6))
        self.errors[-1][:3] = np.dot(R_body, self.desired_state[:3] - current_state[:3])
        d_pos_error = (self.errors[-1][:3] - self.errors[-2][:3]) / self.dt

        desired_attitude = np.zeros(3)

        desired_attitude[:2] = self.attitude_controller.update(self.errors[-1][:2], d_pos_error[:2], self.dt)
        desired_phi = (-1/self.g) * (self.desired_state[7] + desired_attitude[1])
        desired_theta = (1/self.g) * (self.desired_state[6] + desired_attitude[0])
        desired_attitude[0], desired_attitude[1]  = desired_phi, desired_theta
        desired_attitude[2] = self.yaw_controller.update(np.array([0]), np.array([0]), self.dt)[0]

        self.errors[-1][3:6] = desired_attitude - np.dot(R_body, current_state[3:6])
        d_angle_error = (self.errors[-1][3:6] - self.errors[-2][3:6]) / self.dt

        force = self.force_controller.update(np.array([self.errors[-1][2]]), np.array([d_pos_error[2]]), self.dt)
        force = self.mass * (self.g + self.desired_state[8] + force) / (np.cos(current_state[3]) * np.cos(current_state[4]))
        moment = self.moment_controller.update(self.errors[-1][3:6], d_angle_error, self.dt)

        motor_speeds = self.compute_motor_speeds(force[0], moment)

        self.throttle_values = motor_speeds

def main(args=None):
    rclpy.init(args=args)
    node = ThrottlePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()