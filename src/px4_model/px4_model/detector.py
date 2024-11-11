import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import SensorCombined, VehicleGlobalPosition, SensorGps
from std_msgs.msg import Float32MultiArray

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from failure_detection_model.model import MotorFailureDetectionModel

class FailureDetector(Node):
    def __init__(self, file_path):
        super().__init__('failure_detector')

        # Parameters
        self.frequency = 10
        self.sequence_length = 10
        self.input_data = []
        
        # Initialize the model
        
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
        self.failure_detection_publisher = self.create_publisher(Float32MultiArray, '/failure_detection', qos_profile)
        
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
            # Prepare input data
            
            # Reset data storage
            self.sensor_combined_data = None
            self.vehicle_gps_position_data = None
            self.vehicle_global_position_data = None

def main(args=None):
    rclpy.init(args=args)
    file_path = 'src/px4_model/models/Failure-Detection-0.h5'
    failure_detector = FailureDetector(file_path)
    rclpy.spin(failure_detector)
    failure_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()