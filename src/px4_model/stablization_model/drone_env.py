# drone_env.py

from px4_msgs.msg import SensorCombined, SensorGps, VehicleGlobalPosition
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import Empty
import rospy
import numpy as np
from gym import spaces

class DroneEnv(gym.Env):
    def __init__(self, env_config):
        super(DroneEnv, self).__init__()
        
        # Initialize ROS node
        rospy.init_node('drone_env', anonymous=True)
        
        # Configurations from YAML
        self.target_height = env_config.get("target_height", 10.2)
        self.height_tolerance = env_config.get("height_tolerance", 0.2)
        self.max_episode_steps = env_config.get("max_episode_steps", 500)
        self.current_step = 0
        self.base_alt = 486.0

        # Define action and observation space
        # Actions are continuous throttle values for four motors between 0 and 1
        self.action_space = spaces.Box(
            low=0.0, high=1.0, shape=(4,), dtype=np.float32
        )

        # Observations include accelerometer, gyroscope, altitude, and GPS position
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32
        )

        # ROS Subscribers for sensor data specific to PX4 IRIS
        rospy.Subscriber("/fmu/out/sensor_combined", SensorCombined, self._imu_callback)
        rospy.Subscriber("/fmu/out/vehicle_gps_position", SensorGps, self._gps_callback)
        rospy.Subscriber("/fmu/out/vehicle_global_position", VehicleGlobalPosition, self._altitude_callback)

        # ROS Publisher for throttle control (adjust topic if needed)
        self.throttle_pub = rospy.Publisher("/drone/throttle_values", Float32MultiArray, queue_size=1)

        # Internal state for sensor values
        self.imu_data = None
        self.gps_data = None
        self.altitude = 0.0

    def reset(self):
        # Reset simulation environment in Gazebo
        rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()

        # Wait for sensor data
        rospy.sleep(1)
        self.current_step = 0

        # Return the initial observation
        return self._get_observation()

    def step(self, action):
        # Publish throttle values to drone
        throttle_values = Float32MultiArray()
        throttle_values.data = action
        self.throttle_pub.publish(throttle_values)

        # Wait for the action to take effect
        rospy.sleep(0.005)

        # Get next observation
        obs = self._get_observation()

        # Calculate reward and check if done
        reward = self._calculate_reward()
        done = self._is_done()

        self.current_step += 1
        info = {}  # Additional debug info if needed

        return obs, reward, done, info

    def _imu_callback(self, data):
        self.imu_data = data

    def _gps_callback(self, data):
        self.gps_data = data

    def _altitude_callback(self, data):
        # PX4 provides altitude directly in Altitude message
        self.altitude = data.alt - self.base_alt

    def _get_observation(self):
        # Ensure all data is available
        if self.imu_data is None or self.gps_data is None:
            return np.zeros(12, dtype=np.float32)

        # IMU data (accelerometer and gyroscope)
        accel = [
            self.imu_data.accelerometer_m_s2[0],
            self.imu_data.accelerometer_m_s2[1],
            self.imu_data.accelerometer_m_s2[2],
        ]
        gyro = [
            self.imu_data.gyro_rad[0],
            self.imu_data.gyro_rad[1],
            self.imu_data.gyro_rad[2],
        ]

        # GPS data (latitude, longitude, altitude)
        gps_position = [
            self.gps_data.latitude_deg,
            self.gps_data.longitude_deg,
            self.gps_data.altitude_msl_m,
        ]

        # Altitude from VehicleGlobalPosition topic
        baro_altitude = [self.altitude]

        # Combine all sensor readings into a single observation vector
        observation = np.array(accel + gyro + gps_position + baro_altitude, dtype=np.float32)
        return observation

    def _calculate_reward(self):
        # Reward based on height stability
        height_error = abs(self.altitude - self.target_height)
        if height_error < self.height_tolerance:
            reward = 1.0
        else:
            reward = -height_error

        return reward

    def _is_done(self):
        # Check if episode is done
        # Done if max steps reached or drone is too far from target height
        if self.current_step >= self.max_episode_steps:
            return True
        if abs(self.altitude - self.target_height) > 2 * self.height_tolerance:
            return True
        return False

    def render(self, mode="human"):
        # Implement rendering if necessary
        pass

    def close(self):
        # Cleanup if necessary
        rospy.signal_shutdown("Environment closed.")
