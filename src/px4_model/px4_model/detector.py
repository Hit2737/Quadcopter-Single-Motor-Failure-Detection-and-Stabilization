import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import SensorCombined, VehicleGlobalPosition, SensorGps
from std_msgs.msg import Float32MultiArray

import torch
from px4_model.detection_model import MotorFailureDetectionModel
from torch.nn.utils.rnn import pad_sequence, pack_padded_sequence, pad_packed_sequence
import joblib

class FailureDetector(Node):
    def __init__(self, file_path, scaler_path):
        super().__init__('failure_detector')

        # Parameters
        self.frequency = 10
        self.sequence_length = 10
        self.input_data = []
        
        # Initialize the model
        input_size = 16
        hidden_size = 64
        num_layers = 3
        output_size = 5
        dropout_prob = 0.3
        self.model = MotorFailureDetectionModel(input_size, hidden_size, num_layers, output_size, dropout_prob)
        self.model.load_model(file_path)
        self.model.eval()
        
        # Load the StandardScaler
        self.scaler = joblib.load(scaler_path)
        
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
            # Combine the data into a single input tensor
            while len(self.input_data) >= self.sequence_length:
                self.input_data.pop(0)
            combined_data = self.sensor_combined_data[1:] + self.vehicle_gps_position_data[1:] + self.vehicle_global_position_data[1:]
            self.input_data.append(combined_data)
            
            # Normalize the input data
            input_data_normalized = self.scaler.transform(self.input_data)
            input_tensor = torch.tensor(input_data_normalized, dtype=torch.float32).unsqueeze(0)
            
            # Make a prediction
            with torch.no_grad():
                lengths = torch.tensor([input_tensor.shape[1]])
                output = self.model(input_tensor, lengths)
                prediction = (output > 0.5).float().cpu().numpy()

            # Extract is_failure and motor_num
            is_failure = prediction[0][-1][0]
            motor_num = 0
            if is_failure:
                for i in range(1, 5):
                    if prediction[0][-1][i]:
                        motor_num = i
                        break
            
            # Print the prediction
            print(f"Prediction: [is_failure: {is_failure}, motor_num: {motor_num}]")
            
            # Publish the prediction
            prediction_msg = Float32MultiArray(data=[is_failure, motor_num])
            self.failure_detection_publisher.publish(prediction_msg)
            
            # Reset data storage
            self.sensor_combined_data = None
            self.vehicle_gps_position_data = None
            self.vehicle_global_position_data = None

def main(args=None):
    rclpy.init(args=args)
    file_path = 'src/px4_model/models/Failure-Detection-0.pth'
    scaler_path = 'src/px4_model/models/scaler.pkl'
    failure_detector = FailureDetector(file_path, scaler_path)
    rclpy.spin(failure_detector)
    failure_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()