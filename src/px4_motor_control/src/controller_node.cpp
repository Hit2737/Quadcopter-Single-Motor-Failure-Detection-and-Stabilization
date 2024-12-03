/****************************************************************************
 *
 *   Copyright (c) 2023, SMART Research Group, Saxion University of
 *   Applied Sciences.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "px4_offboard_lowlevel/controller_node.h"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/int32.hpp>

ControllerNode::ControllerNode()
    : Node("controller_node")
{
    // Load parameters and matrices
    load_setup_parameters_and_matrices();
    RCLCPP_INFO(this->get_logger(), "Controller Node has been started ...");

    // Defining the compatible ROS 2 predefined QoS for PX4 topics
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Subscribers
    vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(odometry_topic_, qos, std::bind(&ControllerNode::vehicle_odometry_callback, this, _1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(status_topic_, qos, std::bind(&ControllerNode::vehicle_status_callback, this, _1));
    command_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(command_pose_topic_, qos, std::bind(&ControllerNode::command_pose_callback, this, _1));
    failure_detection_sub_ = this->create_subscription<std_msgs::msg::Int32>(failure_detection_topic_, qos, std::bind(&ControllerNode::failure_detection_callback, this, _1));
    accelerometer_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos, std::bind(&ControllerNode::accelerometer_callback, this, _1));

    // Publishers
    actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(actuator_control_topic_, 10);
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_control_topic_, 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic_, 10);

    // Parameters subscriber
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ControllerNode::parameters_callback, this, std::placeholders::_1));

    // Timers
    time_step_ = 0.01;
    std::chrono::duration<double> offboard_period(0.33);
    std::chrono::duration<double> controller_period(time_step_);
    offboardTimer = this->create_wall_timer(offboard_period, [=]()
                                            { offboard_control_mode_publisher(); });
    controllerTimer = this->create_wall_timer(controller_period, [=]()
                                              { update_control_loop(); });
}

// Function to update allocation matrix when a motor fails (for user ease)
rcl_interfaces::msg::SetParametersResult ControllerNode::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    for (const auto &param : parameters)
    {
        if (param.get_name() == "failed_motor")
        {
            int new_value = param.as_int();
            if (0 <= new_value && new_value <= 4)
            {
                result.successful = true;
                result.reason = "success";
                failed_motor_.store(new_value);
                RCLCPP_INFO(this->get_logger(), "Updated failed_motor_ = [%d]", new_value);
                update_allocation_matrix(new_value);
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid value for failed_motor_";
            }
        }
    }
    return result;
}

// Function to update the control allocation matrix when a motor fails
void ControllerNode::update_allocation_matrix(int failed_motor_)
{
    const double kDegToRad = M_PI / 180.0;
    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust;
    Eigen::MatrixXd mixing_matrix;

    const double kS = std::sin(45 * kDegToRad);
    rotor_velocities_to_torques_and_thrust.resize(4, 4);
    mixing_matrix.resize(4, 4);

    rotor_velocities_to_torques_and_thrust << -kS, kS, kS, -kS,
        -kS, kS, -kS, kS,
        -1, -1, 1, 1,
        1, 1, 1, 1;

    mixing_matrix << -0.495384, -0.707107, -0.765306, 1.0,
        0.495384, 0.707107, -1.0, 1.0,
        0.495384, -0.707107, 0.765306, 1.0,
        -0.495384, 0.707107, 1.0, 1.0;

    torques_and_thrust_to_rotor_velocities_.resize(4, 4);
    throttles_to_normalized_torques_and_thrust_.resize(4, 4);

    throttles_to_normalized_torques_and_thrust_ << -0.5718, 0.4376, 0.5718, -0.4376,
        -0.3536, 0.3536, -0.3536, 0.3536,
        -0.2832, -0.2832, 0.2832, 0.2832,
        0.2500, 0.2500, 0.2500, 0.2500;

    Eigen::Vector4d k;
    k << _thrust_constant * _arm_length,
        _thrust_constant * _arm_length,
        _moment_constant * _thrust_constant,
        _thrust_constant;
    rotor_velocities_to_torques_and_thrust = k.asDiagonal() * rotor_velocities_to_torques_and_thrust;

    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust_updated(3, 3);

    int _i = 0;
    for (int i = 0; i < 4; i++)
    {
        if (i == 2)
            continue;

        int _j = 0;
        for (int j = 0; j < 4; j++)
        {
            if (j == failed_motor_ - 1)
                continue;

            rotor_velocities_to_torques_and_thrust_updated(_i, _j) = rotor_velocities_to_torques_and_thrust(i, j);
            ++_j;
        }
        ++_i;
    }

    Eigen::Matrix3d G;
    G << rotor_velocities_to_torques_and_thrust_updated(0, 0) / _inertia_matrix(0), rotor_velocities_to_torques_and_thrust_updated(0, 1) / _inertia_matrix(0), rotor_velocities_to_torques_and_thrust_updated(0, 2) / _inertia_matrix(0),
        rotor_velocities_to_torques_and_thrust_updated(1, 0) / _inertia_matrix(1), rotor_velocities_to_torques_and_thrust_updated(1, 1) / _inertia_matrix(1), rotor_velocities_to_torques_and_thrust_updated(1, 2) / _inertia_matrix(1),
        rotor_velocities_to_torques_and_thrust_updated(2, 0) / _uav_mass, rotor_velocities_to_torques_and_thrust_updated(2, 1) / _uav_mass, rotor_velocities_to_torques_and_thrust_updated(2, 2) / _uav_mass;

    G_inv = G.completeOrthogonalDecomposition().pseudoInverse();

    torques_and_thrust_to_rotor_velocities_updated_.resize(3, 3);
    torques_and_thrust_to_rotor_velocities_updated_ =
        rotor_velocities_to_torques_and_thrust_updated.completeOrthogonalDecomposition().pseudoInverse();

    RCLCPP_INFO(this->get_logger(), "Updated Allocation Matrix for failed motor %d", failed_motor_);
}

// Function to calculate throttles when no motor fails
void ControllerNode::px4_inverse_not_failed(Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench)
{
    Eigen::VectorXd omega;
    Eigen::VectorXd ones_temp;

    omega.resize(4);
    omega.setZero();
    throttles->resize(4);
    throttles->setZero();
    ones_temp.resize(4);
    ones_temp = Eigen::VectorXd::Ones(4, 1);
    omega = torques_and_thrust_to_rotor_velocities_ * (*wrench);

    omega = omega.cwiseSqrt();
    *throttles = (omega - (_zero_position_armed * ones_temp));
    *throttles /= (_input_scaling);

    throttles_prev = *throttles;
}

// Function to calculate throttles when a motor fails
void ControllerNode::px4_inverse_failed(Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench, Eigen::Vector3d *v_in, Eigen::Vector3d *y_)
{
    Eigen::VectorXd omega;
    Eigen::VectorXd ones_temp;

    omega.resize(4);
    omega.setZero();
    throttles->resize(4);
    throttles->setZero();
    ones_temp.resize(4);
    ones_temp = Eigen::VectorXd::Ones(4, 1);
    Eigen::VectorXd modified_wrench(3);
    modified_wrench << (*wrench)(0), (*wrench)(1), (*wrench)(3);

    Eigen::VectorXd omega_temp(3);
    omega.setZero();
    omega_temp = torques_and_thrust_to_rotor_velocities_updated_ * (modified_wrench);
    for (int i = 0, _i = 0; i < 4; i++)
    {
        if (i == failed_motor_ - 1)
            continue;
        omega(i) = omega_temp(_i);
        _i++;
    }

    omega = omega.cwiseSqrt();
    *throttles = (omega - (_zero_position_armed * ones_temp));
    *throttles /= (_input_scaling);

    // Calculate motor speeds
    Eigen::Vector3d y_dot = (*y_ - y_prev_) / time_step_;
    Eigen::VectorXd intermediate = G_inv * (*v_in - y_dot);
    throttles->setZero();
    for (int i = 0, _i = 0; i < 4; i++)
    {
        if (i == failed_motor_ - 1)
            continue;
        (*throttles)(i) = intermediate(_i) + throttles_prev(_i);
        _i++;
    }

    // Update previous values
    throttles_prev = *throttles;
    y_prev_ = *y_;
}

// Function to publish vehicle command
void ControllerNode::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

// Function to publish offboard control mode
void ControllerNode::offboard_control_mode_publisher()
{
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.position = false;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.body_rate = false;

    offboard_msg.attitude = false;
    offboard_msg.thrust_and_torque = false;
    offboard_msg.direct_actuator = true;

    offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(offboard_msg);
}

// Function to get the pose message and update desired position and orientation
void ControllerNode::command_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    eigenTrajectoryPointFromPoseMsg(pose_msg, position, orientation);
    controller_.setTrajectoryPoint(position, orientation);

    RCLCPP_INFO_ONCE(get_logger(), "Controller got first command message.");
}

// Function to get the failure detection message and update the failed motor
void ControllerNode::failure_detection_callback(const std_msgs::msg::Int32::SharedPtr fail_msg)
{
    int failed_motor_temp = fail_msg->data;
    if (failed_motor_.load() == 0 && 0 < failed_motor_temp && failed_motor_temp <= 4)
    {
        update_allocation_matrix(failed_motor_temp);
        failed_motor_.store(failed_motor_temp);
        RCLCPP_INFO(this->get_logger(), "Motor %d failure detected. Applying Safe Control ...", failed_motor_temp);
    }
}

// Function to get the odometry message and update the controller odometry
void ControllerNode::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first odometry message.");

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;

    eigenOdometryFromPX4Msg(odom_msg,
                            position, orientation, velocity, angular_velocity);

    controller_.setOdometry(position, orientation, velocity, angular_velocity);
}

// Function to get the vehicle status message and update the current status (arming and offboard mode)
void ControllerNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg)
{
    current_status_ = *status_msg;
    if (current_status_.arming_state == 2)
    {
        RCLCPP_INFO_ONCE(get_logger(), "ARMED - vehicle_status_msg.");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "NOT ARMED - vehicle_status_msg. Arm command sending ...");
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    }
    if (current_status_.nav_state == 14)
    {
        RCLCPP_INFO_ONCE(get_logger(), "OFFBOARD - vehicle_status_msg.");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "NOT OFFBOARD - vehicle_status_msg. Offboard command sending ...");
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    }
}

// Function to get the accelerometer message and update the controller accelerometer data
void ControllerNode::accelerometer_callback(const px4_msgs::msg::SensorCombined::SharedPtr accel_msg)
{
    Eigen::Vector3d accelerometer_data;
    accelerometer_data << accel_msg->accelerometer_m_s2[0], accel_msg->accelerometer_m_s2[1], accel_msg->accelerometer_m_s2[2];
    controller_.setAccelerometerData(accelerometer_data);
}

// Function to publish the actuator motors message
void ControllerNode::actuator_motors_publisher(const Eigen::VectorXd &throttles)
{
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;
    actuator_motors_msg.control = {(float)throttles[0], (float)throttles[1], (float)throttles[2], (float)throttles[3],
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")};
    actuator_motors_msg.reversible_flags = 0;
    actuator_motors_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
    actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

    actuator_motors_publisher_->publish(actuator_motors_msg);
}

// Function to update the control loop
void ControllerNode::update_control_loop()
{
    //  Compute thrust and torque
    Eigen::VectorXd wrench;
    Eigen::Vector3d v_in;
    Eigen::Vector3d y_;
    Eigen::Quaterniond desired_quaternion;
    controller_.compute_thrust_and_torque(&wrench, &v_in, &y_, &desired_quaternion, time_step_);

    //  calculate throttles
    Eigen::VectorXd throttles;
    if (failed_motor_.load() != 0)
    {
        px4_inverse_failed(&throttles, &wrench, &v_in, &y_);
    }
    else
        px4_inverse_not_failed(&throttles, &wrench);

    // Publish the controller output
    if (current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
        actuator_motors_publisher(throttles);
    }
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}