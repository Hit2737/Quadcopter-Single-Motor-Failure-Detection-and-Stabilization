#!/usr/bin/env python3

import os
import csv
import random
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import SensorCombined, SensorGps, VehicleGlobalPosition

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

class DronePathPlanner:
    def __init__(self, drone: System):
        self.drone = drone
        self.max_latitude = 50.0
        self.min_latitude = 49.5
        self.max_longitude = -0.5
        self.min_longitude = -1.0
        self.max_altitude = 100
        self.min_altitude = 10

    def generate_random_waypoints(self, num_waypoints=5):
        """Generates random waypoints for the drone to follow."""
        waypoints = []
        for _ in range(num_waypoints):
            lat = random.uniform(self.min_latitude, self.max_latitude)
            lon = random.uniform(self.min_longitude, self.max_longitude)
            alt = random.uniform(self.min_altitude, self.max_altitude)
            waypoints.append((lat, lon, alt))
        return waypoints

    async def follow_random_path(self, num_waypoints=5):
        """Follows a random path by generating random waypoints."""
        waypoints = self.generate_random_waypoints(num_waypoints)

        # Arm and take off
        await self.drone.action.arm()
        await self.drone.action.takeoff()

        # Wait for the drone to be airborne
        await asyncio.sleep(5)

        # Move to each waypoint in the path
        for waypoint in waypoints:
            lat, lon, alt = waypoint
            print(f"Going to waypoint: Lat {lat}, Lon {lon}, Alt {alt}")

            # Command the drone to fly to the waypoint
            await self.fly_to_waypoint(lat, lon, alt)
            await asyncio.sleep(5)

        # Return to home after completing the path
        print("Returning to home.")
        await self.drone.action.return_to_launch()

    async def fly_to_waypoint(self, latitude, longitude, altitude):
        """Commands the drone to fly to a given waypoint."""
        # Convert latitude, longitude to NED (North, East, Down) coordinates
        # For simplicity, this example assumes position in meters
        position = PositionNedYaw(latitude, longitude, altitude, 0)
        await self.drone.offboard.set_position_ned(position)

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # Define parameters
        self.n_iterations = 200
        self.frequency = 200
        self.motor_failure = 0
        self.epsilon = 0.5
        self.lock = threading.Lock()
        
        # Create dataset directory if it doesn't exist
        os.makedirs('src/px4_model/dataset', exist_ok=True)
        
        # Open a single CSV file for writing
        self.file_name = None
        self.data_file = None
        self.data_writer = None
        self.max_write = 1000
        self.row_count = 0
        
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

    def setup_data_file(self, file_name):
        # Open a CSV file for writing
        self.file_name = file_name
        self.data_file = open(f'src/px4_model/dataset/{file_name}', mode='w')
        self.data_writer = csv.writer(self.data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.row_count = 0

        # Write the header row
        self.data_writer.writerow([
            'timestamp',
            'gyro_rad[0]', 'gyro_rad[1]', 'gyro_rad[2]',
            'accelerometer_m_s2[0]', 'accelerometer_m_s2[1]', 'accelerometer_m_s2[2]',
            'latitude_deg', 'longitude_deg', 'altitude_msl_m', 'vel_m_s', 'vel_n_m_s', 'vel_e_m_s', 'vel_d_m_s',
            'lat', 'lon', 'alt', 'motor_failure'
        ])
        
    def write_data(self):
        with self.lock:
            if self.data_writer and (self.row_count < self.max_write) and self.sensor_combined_data and self.vehicle_gps_position_data and self.vehicle_global_position_data:
                # Write a row with all data
                self.data_writer.writerow(
                    self.sensor_combined_data + 
                    self.vehicle_gps_position_data[1:] + 
                    self.vehicle_global_position_data[1:] +
                    [self.motor_failure]
                )

                # Reset data storage
                self.sensor_combined_data = None
                self.vehicle_gps_position_data = None
                self.vehicle_global_position_data = None

                # Increment row count
                self.row_count += 1

    def delete_data_file(self):
        # Delete the CSV file
        if self.data_file:
            self.data_writer = None
            self.data_file.close()
            self.data_file = None
            os.remove(f'src/px4_model/dataset/{self.file_name}')

            self.get_logger().info(f"Deleted file: {self.file_name}")

    def close_data_file(self):
        # Close CSV file
        with self.lock:
            self.data_writer = None
            if self.data_file:
                self.data_file.close()
                self.data_file = None

        # Reset data storage
        self.sensor_combined_data = None
        self.vehicle_gps_position_data = None
        self.vehicle_global_position_data = None

    def random_motor_failure(self):
        # Fail a random motor in Gazebo
        has_failed = random.random()

        if has_failed < self.epsilon:
            motor_num = random.randint(1, 4)
            command = f"gz topic -p /gazebo/motor_failure_num -m 'data: {motor_num}'"
            subprocess.run(command, shell=True)
            self.get_logger().info(f"Failing Motor: {motor_num}")

            # Set the motor failure
            self.motor_failure = motor_num
        else:
            self.get_logger().info("No motor failure")

            # Reset the motor failure
            self.motor_failure = 0

    def start_environment(self):
        self.get_logger().info("Starting PX4 SITL environment...")

        # Reset Motor Failure
        self.motor_failure = 0

        # Start PX4 SITL
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'cd ../PX4-Autopilot; make px4_sitl gazebo-classic_iris; exec bash'])

    def stop_environment(self):
        self.get_logger().info("Stopping PX4 SITL environment...")

        # Stop PX4 SITL
        os.system("pkill -f 'make px4_sitl'")

    async def check_connection(self, drone):
        async for state in drone.core.connection_state():
            if state.is_connected:
                return
            
    async def check_health(self, drone):
        async for health in drone.telemetry.health():
            if all([
                health.is_gyrometer_calibration_ok,
                health.is_accelerometer_calibration_ok,
                health.is_magnetometer_calibration_ok,
                health.is_local_position_ok,
                health.is_global_position_ok
            ]):
                return
            
    async def schedule_motor_failure(self, delay):
        try:
            await asyncio.sleep(delay)
            self.random_motor_failure()
        except asyncio.CancelledError:
            pass

    async def timer_task(self):
        # Wait for 30 seconds and then stop everything
        await asyncio.sleep(30)
        self.delete_data_file()
        self.stop_environment()
        self.motor_failure = 0
        await asyncio.sleep(1)

        self.get_logger().info("\n---------------------------------------------------------------\n")

    async def simulation(self):
        try:
            for i in range(self.n_iterations):
                self.get_logger().info(f"Iteration: {i+1}")

                # Start the PX4 SITL environment
                self.start_environment()
                self.setup_data_file(f'DATA_{i}.csv')
                random_failure_time = random.uniform(0, 25)
                motor_failure_task = asyncio.create_task(self.schedule_motor_failure(random_failure_time))
                timer_task = asyncio.create_task(self.timer_task())

                # Connect to the drone
                drone = System()
                try:
                    # Check for connection within 3 seconds
                    await asyncio.wait_for(self.check_connection(drone), timeout=3)
                    
                    # Check if the drone is ready to arm within 3 seconds
                    await asyncio.wait_for(self.check_health(drone), timeout=3)

                    # Plan a random path for the drone to follow
                    path_planner = DronePathPlanner(drone)
                    await path_planner.follow_random_path(num_waypoints=5)

                except asyncio.TimeoutError:
                    self.get_logger().error("Drone connection or health check timed out.")
                    
                    self.delete_data_file()
                    motor_failure_task.cancel()
                    self.stop_environment()

                    self.get_logger().info("\n---------------------------------------------------------------\n")
                    continue

                await motor_failure_task

                await timer_task

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