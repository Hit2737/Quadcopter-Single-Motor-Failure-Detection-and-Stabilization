#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import SensorCombined, SensorGps, VehicleGlobalPosition
import csv
import os
import asyncio
from mavsdk import System
import random
import threading
import subprocess

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # Define parameters
        self.n_iterations = 200
        self.frequency = 10
        self.failed_motor = 0
        self.motor_failure_flag = 0
        self.lock = threading.Lock()
        
        # Create dataset directory if it doesn't exist
        os.makedirs('src/px4_model/dataset', exist_ok=True)
        
        # Open a single CSV file for writing
        self.data_file = None
        self.data_writer = None
        
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
        
        # Initialize data storage
        self.sensor_combined_data = None
        self.vehicle_gps_position_data = None
        self.vehicle_global_position_data = None
        
        # Create a timer to call write_data at 10Hz
        self.timer = self.create_timer(1/self.frequency, self.write_data)
        
    def sensor_combined_callback(self, msg):
        with self.lock:
            self.sensor_combined_data = [msg.timestamp] + list(msg.gyro_rad) + list(msg.accelerometer_m_s2)
        
    def vehicle_gps_position_callback(self, msg):
        with self.lock:
            self.vehicle_gps_position_data = [msg.timestamp, msg.latitude_deg, msg.longitude_deg, msg.altitude_msl_m, msg.vel_m_s, msg.vel_n_m_s, msg.vel_e_m_s, msg.vel_d_m_s]
        
    def vehicle_global_position_callback(self, msg):
        with self.lock:
            self.vehicle_global_position_data = [msg.timestamp, msg.lat, msg.lon, msg.alt]

    def setup_data_file(self, filename):
        # Open a CSV file for writing
        self.data_file = open(f'src/px4_model/dataset/{filename}', mode='w')
        self.data_writer = csv.writer(self.data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        # Write the header row
        self.data_writer.writerow([
            'timestamp',
            'gyro_rad[0]', 'gyro_rad[1]', 'gyro_rad[2]',
            'accelerometer_m_s2[0]', 'accelerometer_m_s2[1]', 'accelerometer_m_s2[2]',
            'latitude_deg', 'longitude_deg', 'altitude_msl_m', 'vel_m_s', 'vel_n_m_s', 'vel_e_m_s', 'vel_d_m_s',
            'lat', 'lon', 'alt', 'motor_failure_flag', 'failed_motor'
        ])
        
    def write_data(self):
        with self.lock:
            if self.data_writer and self.sensor_combined_data and self.vehicle_gps_position_data and self.vehicle_global_position_data:
                # Write a row with all data
                self.data_writer.writerow(
                    self.sensor_combined_data + 
                    self.vehicle_gps_position_data[1:] + 
                    self.vehicle_global_position_data[1:] +
                    [self.motor_failure_flag] +
                    [self.failed_motor]
                )
                # Reset data storage
                self.sensor_combined_data = None
                self.vehicle_gps_position_data = None
                self.vehicle_global_position_data = None
    
    def delete_data_file(self):
        with self.lock:
            # Close CSV file
            self.data_writer = None
            self.data_file.close()
            self.data_file = None
    
    def run_px4_simulation(self):
        print("Starting PX4 SITL with Gazebo Classic...")
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'cd ../PX4-Autopilot; make px4_sitl gazebo-classic_iris; exec bash'])

    def stop_motor_in_gazebo(self, motor_num):
        command = f"gz topic -p /gazebo/motor_failure_num -m 'data: {motor_num}'"
        print(f"Stopping motor {motor_num} in Gazebo...")
        subprocess.run(command, shell=True)

    def set_motor_failure(self):
        with self.lock:
            self.motor_failure_flag = 1

    def reset_motor_failure(self):
        with self.lock:
            self.motor_failure_flag = 0

    def close_px4_terminal(self):
        print("Closing PX4 terminal...")
        os.system("pkill -f 'make px4_sitl'")

    async def simulation(self):
        try:
            for i in range(68,self.n_iterations):
                print(f"Iteration: {i+1}")
                self.run_px4_simulation()
                self.setup_data_file(f'DATA_{i}.csv')
                self.get_logger().info("PX4 SITL started.")

                drone = System()
                await drone.connect()
                async for state in drone.core.connection_state():
                    if state.is_connected:
                        break

                async for health in drone.telemetry.health():
                    if all([health.is_gyrometer_calibration_ok, health.is_accelerometer_calibration_ok,
                            health.is_magnetometer_calibration_ok, health.is_local_position_ok,
                            health.is_global_position_ok]):
                        print("Drone is ready to arm")
                        break

                try:
                    await drone.action.arm()
                    await drone.action.takeoff()
                    print("Takeoff Detected")
                except Exception as e:
                    print(f"Drone action failed: {e}")
                    return
                await asyncio.sleep(10 + random.randint(0, 10))

                motor_num = random.randint(1, 4)
                self.stop_motor_in_gazebo(motor_num)
                self.set_motor_failure()
                print(f"Failing Motor: {motor_num}")
                self.failed_motor = motor_num
                await asyncio.sleep(5)
                self.delete_data_file()
                self.reset_motor_failure()
                self.close_px4_terminal()
                self.failed_motor = 0

                await asyncio.sleep(1)

                print("\n---------------------------------------------------------------\n")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)

    data_collector = DataCollector()

    # Run the simulation in a separate thread
    simulation_thread = threading.Thread(target=lambda: asyncio.run(data_collector.simulation()), daemon=True)
    simulation_thread.start()

    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(data_collector)

    data_collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()