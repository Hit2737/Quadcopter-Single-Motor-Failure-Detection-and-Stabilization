import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import SensorCombined, VehicleGlobalPosition, SensorGps
from std_msgs.msg import Int32

# from tensorflow.keras.models import load_model

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class FailureDetector(Node):
    def __init__(self, frequency=100):
        super().__init__('failure_detector')

        # Parameters
        self.frequency = frequency
        self.input_data = []
        
        # Initialize the model
        # self.model = load_model(file_path)
        # self.sequence_length = self.model.input_shape[1]
        
        self.gyro_flag = False
        self.acc_flag = False
        self.vel_flag = False
        self.failed_motor = -1

        # Initialize data storage
        self.sensor_combined_data = None
        self.vehicle_gps_position_data = None
        self.vehicle_global_position_data = None
        
        # Define QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers with QoS settings
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_combined_callback, qos_profile)
        self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_position_callback, qos_profile)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)

        # Create a publisher for the failure detection topic
        self.failure_detection_publisher = self.create_publisher(Int32, '/failure_detection', qos_profile)
        
        # Create a timer to call the prediction function at a regular interval
        self.timer = self.create_timer(1/self.frequency, self.predict_failure)
        
    def sensor_combined_callback(self, msg):
        self.sensor_combined_data = [msg.timestamp] + list(msg.gyro_rad) + list(msg.accelerometer_m_s2)
    
    def vehicle_gps_position_callback(self, msg):
        self.vehicle_gps_position_data = [msg.timestamp, msg.latitude_deg, msg.longitude_deg, msg.altitude_msl_m, msg.vel_m_s, msg.vel_n_m_s, msg.vel_e_m_s, msg.vel_d_m_s]
    
    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position_data = [msg.timestamp, msg.lat, msg.lon, msg.alt]
    
    def predict_failure(self):
        if self.sensor_combined_data and self.vehicle_gps_position_data and self.vehicle_global_position_data:

            # Motor 1
            if self.sensor_combined_data[1] > 5 and self.sensor_combined_data[2] < 5 and self.sensor_combined_data[3] < 0:
                self.gyro_flag = True
                self.failed_motor = 1
            # Motor 2
            elif self.sensor_combined_data[1] < -5 and self.sensor_combined_data[2] > 5 and self.sensor_combined_data[3] < 0:
                self.gyro_flag = True
                self.failed_motor = 2
            # Motor 3
            elif self.sensor_combined_data[1] < -5 and self.sensor_combined_data[2] < -5 and self.sensor_combined_data[3] > 0:
                self.gyro_flag = True
                self.failed_motor = 3
            # Motor 4
            elif self.sensor_combined_data[1] > 5 and self.sensor_combined_data[2] > 5 and self.sensor_combined_data[3] > 0:
                self.gyro_flag = True
                self.failed_motor = 4
            
            if abs(self.sensor_combined_data[4]) > 10 or abs(self.sensor_combined_data[5]) > 10 or abs(self.sensor_combined_data[6]) > 20:
                self.acc_flag = True
            if abs(self.vehicle_gps_position_data[7]) > 8:
                self.vel_flag = True
            
            # Failure Detected
            msg = Int32()
            msg.data = -1
            if self.gyro_flag and self.acc_flag and self.vel_flag:
                # Publish the failure
                msg.data = self.failed_motor
            self.failure_detection_publisher.publish(msg)
            
            # Reset data storage
            self.sensor_combined_data = None
            self.vehicle_gps_position_data = None
            self.vehicle_global_position_data = None
            self.failed_motor = -1

def main(args=None):
    rclpy.init(args=args)
    frequency = 100
    failure_detector = FailureDetector(frequency)
    rclpy.spin(failure_detector)
    failure_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()