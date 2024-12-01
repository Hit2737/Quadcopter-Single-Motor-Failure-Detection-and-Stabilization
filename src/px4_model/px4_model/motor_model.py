import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import Float32MultiArray

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import pinv

def rotate_vector_from_to_enu_ned(vec_in):
    vec_out = np.array([vec_in[1], vec_in[0], -vec_in[2]])
    return vec_out

def rotate_quaternion_from_to_enu_ned(quat_in):
    euler_1 = [np.pi, 0.0, np.pi / 2]
    NED_ENU_Q = R.from_euler('zyx', euler_1)

    euler_2 = [np.pi, 0.0, 0.0]
    AIRCRAFT_BASELINK_Q = R.from_euler('zyx', euler_2)

    quat_in = np.array(quat_in)
    input_quat = R.from_quat(quat_in)

    transformed_quat = (NED_ENU_Q * input_quat) * AIRCRAFT_BASELINK_Q
    return transformed_quat

def rotate_vector_from_to_frd_flu(vec_in):
    vec_out = np.array([vec_in[0], -vec_in[1], -vec_in[2]])
    return vec_out

class ThrottlePublisher(Node):
    def __init__(self):
        super().__init__('throttle_publisher')

        # Define QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.mass = 1.725
        self.inertia_matrix = np.array([0.029125, 0.029125, 0.055225])
        self.gravity = 9.81
        self.arm_length = 0.25
        self.moment_constant = 0.06
        self.thrust_constant = 5.84e-06
        self.max_rotor_speed = 1100.0
        self.R_B_W = np.eye(3)

        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        self.position_r = np.array([0, 0, 2])
        self.velocity_r = np.array([0, 0, 0])
        self.acceleration_r = np.array([0, 0, 0])
        self.yaw_r = 0
        self.yaw_rate_r = 0

        self.position_gain = np.array([7.0, 7.0, 6.0])
        self.velocity_gain = np.array([6.0, 6.0, 3.0])
        self.attitude_gain = np.array([3.5, 3.5, 0.3])
        self.angular_rate_gain = np.array([0.5, 0.5, 0.1])
        self.omega_to_pwn_coefficients = np.array([0.001142, 0.2273, 914.2])
        self.pwm_min = 1075
        self.pwm_max = 1950
        self.input_scaling = 1000

        self.compute_control_allocation_and_actuator_matrices()

        # Create subscribers with QoS settings
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        self.throttle_values = [0.0, 0.0, 0.0, 0.0]
        self.dt = 0.005
        self.publisher_ = self.create_publisher(Float32MultiArray, '/drone/throttle_values', 10)
        self.control_timer = self.create_timer(self.dt, self.update_control_loop)
        self.throttle_timer = self.create_timer(self.dt, self.publish_throttle_values)

    def publish_throttle_values(self):
        msg = Float32MultiArray()
        msg.data = [self.throttle_values[0], self.throttle_values[1], self.throttle_values[2], self.throttle_values[3]]
        self.publisher_.publish(msg)

        print("Throttle values: ", self.throttle_values)

    def compute_control_allocation_and_actuator_matrices(self):
        k_deg_to_rad = np.pi / 180.0
        k_s = np.sin(45.0 * k_deg_to_rad)

        self.rotor_velocities_to_torques_and_thrust = np.array([
            [-k_s,  k_s,  k_s, -k_s],
            [-k_s,  k_s, -k_s,  k_s],
            [-1,   -1,    1,    1],
            [ 1,    1,    1,    1]
        ])

        self.mixing_matrix = np.array([
            [-0.495384,  -0.707107,  -0.765306,   1.0],
            [ 0.495384,   0.707107,  -1.0,        1.0],
            [ 0.495384,  -0.707107,   0.765306,   1.0],
            [-0.495384,   0.707107,   1.0,        1.0]
        ])

        self.throttles_to_normalized_torques_and_thrust = np.array([
            [-0.5718,    0.4376,    0.5718,   -0.4376],
            [-0.3536,    0.3536,   -0.3536,    0.3536],
            [-0.2832,   -0.2832,    0.2832,    0.2832],
            [ 0.2500,    0.2500,    0.2500,    0.2500]
        ])

        k = np.diag([
            self.thrust_constant * self.arm_length, 
            self.thrust_constant * self.arm_length,
            self.moment_constant * self.thrust_constant, 
            self.thrust_constant
        ])

        self.rotor_velocities_to_torques_and_thrust = k @ self.rotor_velocities_to_torques_and_thrust
        self.torques_and_thrust_to_rotor_velocities = pinv(self.rotor_velocities_to_torques_and_thrust)

        print("rotor_velocities_to_torques_and_thrust = \n", self.rotor_velocities_to_torques_and_thrust)
        print("torques_and_thrust_to_rotor_velocities = \n", self.torques_and_thrust_to_rotor_velocities)
        print("throttles_to_normalized_torques_and_thrust_ = \n", self.throttles_to_normalized_torques_and_thrust)

    def eigen_odometry_from_px4_msg(self, msg):
        position_W = rotate_vector_from_to_enu_ned(np.array(msg.position))
        quaternion = [msg.q[1], msg.q[2], msg.q[3], msg.q[0]]
        orientation_B_W = rotate_quaternion_from_to_enu_ned(quaternion)
        velocity_B = rotate_vector_from_to_enu_ned(np.array(msg.velocity))
        angular_velocity_B = rotate_vector_from_to_frd_flu(np.array(msg.angular_velocity))

        return position_W, orientation_B_W, velocity_B, angular_velocity_B

    def vehicle_odometry_callback(self, msg):
        position_W, orientation_B_W, velocity_B, angular_velocity_B = self.eigen_odometry_from_px4_msg(msg)
        self.R_B_W = orientation_B_W.as_matrix()
        self.position = position_W
        self.velocity = self.R_B_W.T @ velocity_B
        self.angular_velocity = angular_velocity_B

        if not hasattr(self, 'logged_once'):
            self.get_logger().info('vehicle_odometry_callback is called')
            self.logged_once = True

        print(f"Vehicle Odometry: \nPosition: {self.position}\nOrientation: {self.R_B_W}\nVelocity: {self.velocity}\nAngular Velocity: {self.angular_velocity}")

    def calculate_controller_outputs(self):
        # Compute translational tracking errors
        e_p = self.position - self.position_r
        e_v = self.velocity - self.velocity_r

        I_a_d = (
            -np.multiply(self.position_gain, e_p)
            - np.multiply(self.velocity_gain, e_v)
            + self.mass * self.gravity * np.array([0, 0, 1])
            + self.mass * self.acceleration_r
        )
        thrust = I_a_d.dot(self.R_B_W[:, 2])
        B_z_d = I_a_d / np.linalg.norm(I_a_d)

        # Calculate desired rotation matrix
        B_x_d = np.array([np.cos(self.yaw_r), np.sin(self.yaw_r), 0.0])
        B_y_d = np.cross(B_z_d, B_x_d)
        B_y_d /= np.linalg.norm(B_y_d)

        R_d_w = np.zeros((3, 3))
        R_d_w[:, 0] = np.cross(B_y_d, B_z_d)
        R_d_w[:, 1] = B_y_d
        R_d_w[:, 2] = B_z_d

        # Compute desired quaternion
        desired_quaternion = R.from_matrix(R_d_w).as_quat()  # Quaternion as [x, y, z, w]
        desired_quaternion = desired_quaternion

        # Attitude tracking
        e_R_matrix = 0.5 * (R_d_w.T @ self.R_B_W - self.R_B_W.T @ R_d_w)
        e_R = np.array([e_R_matrix[2, 1], e_R_matrix[0, 2], e_R_matrix[1, 0]])
        omega_ref = self.yaw_rate_r * np.array([0, 0, 1])
        e_omega = self.angular_velocity - self.R_B_W.T @ R_d_w @ omega_ref

        tau = (
            -np.multiply(self.attitude_gain, e_R)
            - np.multiply(self.angular_rate_gain, e_omega)
            + np.cross(
                self.angular_velocity,
                (self.inertia_matrix * self.angular_velocity)
            )
        )

        # Output the wrench
        wrench = np.zeros(4)
        wrench[:3] = tau
        wrench[3] = thrust
        
        return wrench
    
    def inverse_kinematics(self, wrench):
        omega = self.torques_and_thrust_to_rotor_velocities @ wrench
        omega = np.sqrt(np.clip(omega, 0, np.inf))
        throttles = (omega - 100) / (self.input_scaling)

        normalized_throttles = self.throttles_to_normalized_torques_and_thrust @ throttles
        return normalized_throttles, throttles
    
    def update_control_loop(self):
        
        wrench = self.calculate_controller_outputs()
        normalized_throttles, throttles = self.inverse_kinematics(wrench)

        self.throttle_values = throttles

def main(args=None):
    rclpy.init(args=args)
    node = ThrottlePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()