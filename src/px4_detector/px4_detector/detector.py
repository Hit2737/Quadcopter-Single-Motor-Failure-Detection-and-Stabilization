import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import SensorCombined, VehicleGlobalPosition, SensorGps
from std_msgs.msg import Int32, Float32MultiArray


class FailureDetector(Node):
    def __init__(self, frequency=100):
        super().__init__("failure_detector")

        # Parameters
        self.frequency = frequency
        self.input_data = []

        self.gyroXg5 = False
        self.gyroYg3 = False
        self.gyroXl5 = False
        self.gyroYl3 = False
        self.failed_motor = 0
        self.gyro_prev = np.array([0, 0, 0])

        # Define QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Create subscribers with QoS settings
        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.sensor_combined_callback,
            qos_profile,
        )
        self.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            self.vehicle_gps_position_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.vehicle_global_position_callback,
            qos_profile,
        )

        # Create a publisher for the failure detection topic
        self.failure_detection_publisher = self.create_publisher(
            Int32, "/detected_failed_motor", qos_profile
        )

        self.gyro_change = self.create_publisher(
            Float32MultiArray, "/gyro_change", qos_profile
        )

        # Create a timer to call the prediction function at a regular interval
        self.timer = self.create_timer(1 / self.frequency, self.predict_failure)

    def sensor_combined_callback(self, msg):
        # Store the sensor data
        dgyro_dt = (np.array(msg.gyro_rad) - self.gyro_prev) * self.frequency
        self.gyro_prev = np.array(msg.gyro_rad)
        self.gyro_change.publish(Float32MultiArray(data=dgyro_dt))

        if not self.gyroXl5 and msg.gyro_rad[0] > 5:
            self.gyroXg5 = True
        if not self.gyroYl3 and msg.gyro_rad[1] > 3:
            self.gyroYg3 = True
        if not self.gyroXg5 and msg.gyro_rad[0] < -5:
            self.gyroXl5 = True
        if not self.gyroYg3 and msg.gyro_rad[1] < -3:
            self.gyroYl3 = True

        if self.failed_motor == 0:
            if dgyro_dt[0] > 20 and dgyro_dt[1] < -15:
                self.failed_motor = 1
            elif dgyro_dt[0] < -20 and dgyro_dt[1] > 15:
                self.failed_motor = 2
            elif dgyro_dt[0] < -20 and dgyro_dt[1] < -15:
                self.failed_motor = 3
            elif dgyro_dt[0] > 20 and dgyro_dt[1] > 15:
                self.failed_motor = 4

            if self.gyroXg5 and self.gyroYl3:
                self.failed_motor = 1
            elif self.gyroXl5 and self.gyroYg3:
                self.failed_motor = 2
            elif self.gyroXl5 and self.gyroYl3:
                self.failed_motor = 3
            elif self.gyroXg5 and self.gyroYg3:
                self.failed_motor = 4

    def vehicle_gps_position_callback(self, msg):
        self.vehicle_gps_position_data = [
            msg.timestamp,
            msg.latitude_deg,
            msg.longitude_deg,
            msg.altitude_msl_m,
            msg.vel_m_s,
            msg.vel_n_m_s,
            msg.vel_e_m_s,
            msg.vel_d_m_s,
        ]

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position_data = [msg.timestamp, msg.lat, msg.lon, msg.alt]

    def predict_failure(self):
        # Failure Detected
        msg = Int32()
        # Publish the failure
        msg.data = self.failed_motor
        self.failure_detection_publisher.publish(msg)

        # Reset data storage
        self.sensor_combined_data = None
        self.vehicle_gps_position_data = None
        self.vehicle_global_position_data = None


def main(args=None):
    rclpy.init(args=args)
    frequency = 100
    failure_detector = FailureDetector(frequency)
    rclpy.spin(failure_detector)
    failure_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
