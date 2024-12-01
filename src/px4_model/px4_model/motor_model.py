from math import pi
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import SensorCombined, VehicleOdometry
from std_msgs.msg import Float32MultiArray

import numpy as np

def R_a2_theta(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def R_a1_phi(phi):
    return np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])

def R_a3_psi(psi):
    return np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])

def skew_symmetric(vector):
    return np.array([
        [0, -vector[2], vector[1]],
        [vector[2], 0, -vector[0]],
        [-vector[1], vector[0], 0]
    ])

class PIDController:
    def __init__(self, KP, KD, KI, size = 3, integral_limit = None):
        self.KP = KP
        self.KD = KD
        self.KI = KI

        # Initialize Errors
        self.integral = np.zeros(size)
        self.integral_limit = integral_limit

    def update(self, error, derivative_error, dt):
        # Update integral term
        self.integral += error * dt
        
        if self.integral_limit:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        # PID output
        output = self.KP * error + self.KD * derivative_error + self.KI * self.integral

        return output

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
        self.force_controller = PIDController(KP=np.array([0.0]), KD=np.array([0.0]), KI=np.array([0.0]), size=1)
        self.moment_controller = PIDController(KP=np.array([0.001, 0.001, 0.001]), KD=np.array([0.50, 0.50, 0.50]), KI=np.array([0.0, 0.0, 0.0]), size=3)
        self.attitude_controller = PIDController(KP=np.array([0.001, 0.001]), KD=np.array([0.50, 0.50]), KI=np.array([0.0, 0.0]), size=2)
        self.yaw_controller = PIDController(KP=np.array([0.0]), KD=np.array([0.0]), KI=np.array([0.0]), size=1)

        # Initialize data storage
        self.vehicle_odometry_data = None
        self.sensor_combined_data = None
        self.vehicle_global_position_data = None

        # Create subscribers with QoS settings
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_combined_callback, qos_profile)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        self.throttle_values = [0.0, 0.0, 0.0, 0.0]
        self.dt = 0.000005
        self.publisher_ = self.create_publisher(Float32MultiArray, '/drone/throttle_values', 10)
        self.force_moment_publisher = self.create_publisher(Float32MultiArray, '/drone/force_moment_values', 10)
        self.control_timer = self.create_timer(self.dt, self.update_control_loop)
        self.throttle_timer = self.create_timer(self.dt, self.publish_throttle_values)

        # Desired_state = [X, Y, Z, Vx, Vy, Vz, Ax, Ay, Az, Phi, Theta, Psi]
        self.desired_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.errors = [np.zeros(6)]

        self.mass = 1.5
        self.J = np.diag([0.03, 0.03, 0.06])
        self.g = 9.80660
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

        KF = 5.85e-06
        KM = 0.06
        self.allocation_matrix = np.array([
            [KF, KF, KF, KF],
            [-KF * FR_b2, KF * BL_b2, KF * FL_b2, -KF * BR_b2],
            [-KF * FR_b1, KF * BL_b1, -KF * FL_b1, KF * BR_b1],
            [KM, KM, -KM, -KM]
        ])
        print(self.allocation_matrix)

    def publish_throttle_values(self):
        msg = Float32MultiArray()
        msg.data = [self.throttle_values[0], self.throttle_values[1], self.throttle_values[2], self.throttle_values[3]]
        # msg.data = [0.5, 0.5, 0.5, 0.5]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published throttle values: {msg.data}')

    def publish_force_moment_values(self, force, moment):
        msg = Float32MultiArray()
        msg.data = [force[0], moment[0], moment[1], moment[2]]
        self.force_moment_publisher.publish(msg)
        self.get_logger().info(f'Published force and moment values: {msg.data}')

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry_data = [msg.timestamp, msg.position[0], msg.position[1], msg.position[2],
                                    msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.q[0], msg.q[1],
                                    msg.q[2], msg.q[3], msg.angular_velocity[0], msg.angular_velocity[1],
                                    msg.angular_velocity[2]]

    def sensor_combined_callback(self, msg):
        self.sensor_combined_data = [msg.timestamp] + list(msg.gyro_rad) + list(msg.accelerometer_m_s2)

    def compute_body_frame_matrix(self, phi, theta, psi):
        return np.dot(np.dot(R_a3_psi(psi), R_a2_theta(theta)), R_a1_phi(phi))

    def compute_motor_speeds(self, F, M):
        omega_squared = np.linalg.inv(self.allocation_matrix) @ np.array([F, M[0], M[1], M[2]])
        self.omega = np.sqrt(omega_squared)
        self.omega = self.omega / self.max_motor_speed
        return self.omega
    
    def compute_force_moment(self, omega):
        Prod = np.dot(self.allocation_matrix, (omega * self.max_motor_speed)**2)
        F = Prod[0]
        M = Prod[1:]
        return F, M

    def update_control_loop(self):
        if self.sensor_combined_data and self.vehicle_odometry_data:
            current_state = [0] * 6

            current_state[:3] = self.vehicle_odometry_data[1:4]
            current_state[3:6] = self.sensor_combined_data[1:4]

            print(f"Current State: {current_state}")

            BFM = self.compute_body_frame_matrix(current_state[3], current_state[4], current_state[5])
            R_body = BFM[:3, :3]

            self.errors.append(np.zeros(6))
            if len(self.errors) > 2:
                self.errors.pop(0)
            self.errors[-1][:3] = np.dot(R_body, np.array(self.desired_state[:3]) - np.array(current_state[:3]))
            d_pos_error = (self.errors[-1][:3] - self.errors[-2][:3]) / self.dt

            desired_attitude = np.zeros(3)

            desired_attitude[:2] = self.attitude_controller.update(self.errors[-1][:2], d_pos_error[:2], self.dt)
            desired_phi = (-1/self.g) * (self.desired_state[7] + desired_attitude[1])
            desired_theta = (1/self.g) * (self.desired_state[6] + desired_attitude[0])
            desired_attitude[0], desired_attitude[1] = desired_phi, desired_theta
            desired_attitude[2] = self.yaw_controller.update(np.array([0]), np.array([0]), self.dt)[0]

            self.errors[-1][3:6] = desired_attitude - np.dot(R_body, current_state[3:6])
            d_angle_error = (self.errors[-1][3:6] - self.errors[-2][3:6]) / self.dt

            acc = self.force_controller.update(np.array([self.errors[-1][2]]), np.array([d_pos_error[2]]), self.dt)
            print("------------------------------------")
            print(self.desired_state[8]+acc / (np.cos(current_state[3]) * np.cos(current_state[4])))
            print("------------------------------------")
            force = self.mass * (self.g + self.desired_state[8] + acc / (np.cos(current_state[3]) * np.cos(current_state[4])))
            force = [14.7]
            moment = self.moment_controller.update(self.errors[-1][3:6], d_angle_error, self.dt)
            print("--------------------")
            print(f"Desired Attitude: {desired_attitude}, Current Attitude: {np.dot(R_body, current_state[3:6])}")
            print("====================================")
            print("Errors: ", self.errors[-1][3:6])
            motor_speeds = self.compute_motor_speeds(force[0], moment)

            print(f"Force: {force[0]}, Moment: {moment}")
            print(f"Motor Speeds: {motor_speeds}")
            print("====================================")
            print("\n\n")
            # print(f"Errors: {self.errors[-1]}")
            self.publish_force_moment_values(force, moment)
            self.throttle_values = motor_speeds

def main(args=None):
    rclpy.init(args=args)
    node = ThrottlePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()