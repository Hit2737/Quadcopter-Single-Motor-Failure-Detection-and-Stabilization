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

#include <vector>
#include <std_msgs/msg/int32.hpp>
#include "px4_motor_control/controller_node.h"



ControllerNode::ControllerNode() 
    : Node("controller_node")
    {
        load_setup_paramters();
        compute_ControlAllocation_and_ActuatorEffect_matrices();

        // Defining the compatible ROS 2 predefined QoS for PX4 topics
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Subscribers
        vehicle_odometry_sub_= this->create_subscription<px4_msgs::msg::VehicleOdometry>
            (odometry_topic_, qos, std::bind(&ControllerNode::vehicle_odometry_callback, this, _1));
        vehicle_status_sub_= this->create_subscription<px4_msgs::msg::VehicleStatus>
            (status_topic_, qos, std::bind(&ControllerNode::vehicle_status_callback, this, _1));
        command_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (command_pose_topic_, 10, std::bind(&ControllerNode::command_pose_callback, this, _1));
        throttle_values_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>
            (throttle_values_topic_, 10, std::bind(&ControllerNode::throttle_values_callback, this, _1));

        // Publishers
        attitude_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>
            (attitude_setpoint_topic_, 10);  
        actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>
            (actuator_control_topic_, 10);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>
            (offboard_control_topic_, 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>
            (vehicle_command_topic_, 10);
        thrust_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>
            (thrust_setpoint_topic_, 10);
        torque_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>
            (torque_setpoint_topic_, 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>
            (traj_setpoint_topic_, 10);

        // Parameters
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ControllerNode::parameter_callback, this, _1));

        // Timers
        std::chrono::duration<double> offboard_period(0.33);        
        std::chrono::duration<double> controller_period(0.01);        
        offboardTimer = this->create_wall_timer(offboard_period, [=]() {offboard_control_mode_publisher();});
        controllerTimer = this->create_wall_timer(controller_period, [=]() {update_controller_output();});
    }

rcl_interfaces::msg::SetParametersResult ControllerNode::parameter_callback(const std::vector<rclcpp::Parameter> &parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // print info about the changed parameter
    for (const auto &param: parameters){
        RCLCPP_INFO(this->get_logger(), "Parameter %s has changed to [%s]", param.get_name().c_str(), param.value_to_string().c_str());
        if(param.get_name() == "control_mode"){
            control_mode_ = param.as_int();
        }
        else if (param.get_name() == "inject_motor_failure"){
            inject_motor_failure_ = param.as_int();
        }
    }
    return result;
}

void ControllerNode::arm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void ControllerNode::disarm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void ControllerNode::set_rotor_throttles(const std::vector<int>& indices, const std::vector<float>& values) {
    if (indices.size() != values.size()) {
        RCLCPP_WARN(this->get_logger(), "Indices and values arrays must have the same size.");
        return;
    }

    for (size_t i = 0; i < indices.size(); ++i) {
        int index = indices[i];
        float value = values[i];
        if (index >= 0 && index < throttles.size()) {
            throttles[index] = value;
        } else {
            RCLCPP_WARN(this->get_logger(), "Index out of bounds: %d", index);
        }
    }
}

void ControllerNode::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void ControllerNode::offboard_control_mode_publisher()
{
	px4_msgs::msg::OffboardControlMode offboard_msg{};
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;
	offboard_msg.body_rate = false;
    offboard_msg.attitude = false;
    offboard_msg.thrust_and_torque = false;

    if (control_mode_ == 1){
        offboard_msg.position = true;
        offboard_msg.direct_actuator = false;
    }
    else if (control_mode_ == 2){
        offboard_msg.position = false;
        offboard_msg.direct_actuator = true;
    }
    else {
        offboard_msg.position = true;
        offboard_msg.direct_actuator = false;
    }
	offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(offboard_msg);
    RCLCPP_INFO_ONCE(get_logger(),"Offboard enabled");
}

void ControllerNode::command_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {                   // When a command is received
    // initialize vectors
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    eigenTrajectoryPointFromPoseMsg(pose_msg, position, orientation);
    RCLCPP_INFO_ONCE(get_logger(),"Controller got first command message.");
    controller_.setTrajectoryPoint(position, orientation);          // Send the command to controller_ obj
}

void ControllerNode::command_trajectory_callback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr& traj_msg) {                   // When a command is received
    // initialize vectors
    Eigen::Vector3d position;
    Eigen::Vector3d velocity; 
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d acceleration;
    eigenTrajectoryPointFromMsg( traj_msg, position, orientation, velocity, angular_velocity, acceleration);
    controller_.setTrajectoryPoint(position, velocity, acceleration, orientation, angular_velocity);
    RCLCPP_INFO_ONCE(get_logger(),"Controller got first command message.");
}

void ControllerNode::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg){
        //  Debug message
        RCLCPP_INFO_ONCE(get_logger(),"Controller got first odometry message.");

        Eigen::Vector3d position;
        Eigen::Vector3d velocity; 
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angular_velocity;
        
        eigenOdometryFromPX4Msg(odom_msg,
                                position, orientation, velocity, angular_velocity);

        controller_.setOdometry(position, orientation, velocity, angular_velocity);
}

void ControllerNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg){
    current_status_ = *status_msg;
    if (current_status_.arming_state == 2){
        RCLCPP_INFO_ONCE(get_logger(),"ARMED - vehicle_status_msg.");
    }
    else {
        RCLCPP_INFO_ONCE(get_logger(),"NOT ARMED - vehicle_status_msg.");
        // Arm the vehicle
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }
    if (current_status_.nav_state == 14){
        RCLCPP_INFO_ONCE(get_logger(),"OFFBOARD - vehicle_status_msg.");
    }
    else {
        RCLCPP_INFO_ONCE(get_logger(),"NOT OFFBOARD - vehicle_status_msg.");
        // Switch to offboard mode
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
    }
}

void ControllerNode::throttle_values_callback(const std_msgs::msg::Float32MultiArray::SharedPtr throttle_msg){
    if (throttle_msg->data.size() != 4){
        RCLCPP_WARN(this->get_logger(), "Throttle values array must have size 4.");
        return;
    }
    for (size_t i = 0; i < 4; ++i) {
        throttles[i] = throttle_msg->data[i];
    }
    RCLCPP_INFO_ONCE(get_logger(),"Throttle values received.");
}

void ControllerNode::actuator_motor_publisher(const Eigen::VectorXd& throttles) {
    // Lockstep should be disabled from PX4 and from the model.sdf file
    // direct motor throttles control
    // Prepare msg
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;
    actuator_motors_msg.control = { (float) throttles[0], (float) throttles[1], (float) throttles[2], (float) throttles[3], 
                            std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                            std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")};
	actuator_motors_msg.reversible_flags = 0;
	actuator_motors_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
	actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

	actuator_motors_publisher_->publish(actuator_motors_msg);

    // Debug message
    RCLCPP_INFO_ONCE(get_logger(),"Actuator motors message published: [%f, %f, %f, %f]", throttles[0], throttles[1], throttles[2], throttles[3]);
}

void ControllerNode::thrust_torque_publisher(const Eigen::Vector4d& controller_output) {
    // Lockstep should be disabled from PX4 and from the model.sdf file
    // Prepare msgs
    px4_msgs::msg::VehicleThrustSetpoint thrust_sp_msg;
    px4_msgs::msg::VehicleTorqueSetpoint torque_sp_msg;
    thrust_sp_msg.timestamp_sample = this->get_clock()->now().nanoseconds() / 1000;
    torque_sp_msg.timestamp_sample = thrust_sp_msg.timestamp_sample ;
    thrust_sp_msg.timestamp = thrust_sp_msg.timestamp_sample ;
    torque_sp_msg.timestamp = thrust_sp_msg.timestamp_sample ;
    // Fill thrust setpoint msg
    thrust_sp_msg.xyz[0] = 0.0;
    thrust_sp_msg.xyz[1] = 0.0;
    if (controller_output[3] > 0.1){
        thrust_sp_msg.xyz[2] = -controller_output[3];         // DO NOT FORGET THE MINUS SIGN (body NED frame)
    }
    else {
        thrust_sp_msg.xyz[2] = -0.1;
    }
    // Rotate torque setpoints from FLU to FRD and fill the msg
    Eigen::Vector3d rotated_torque_sp;
    rotated_torque_sp = rotateVectorFromToFRD_FLU(Eigen::Vector3d(controller_output[0], controller_output[1], controller_output[2]));
    torque_sp_msg.xyz[0] = rotated_torque_sp[0];
    torque_sp_msg.xyz[1] = rotated_torque_sp[1];
    torque_sp_msg.xyz[2] = rotated_torque_sp[2];

    // Publish msgs
    thrust_setpoint_publisher_->publish(thrust_sp_msg);
    torque_setpoint_publisher_->publish(torque_sp_msg);
}

void ControllerNode::attitute_setpoint_publisher(const Eigen::Vector4d& controller_output, const Eigen::Quaterniond& desired_quaternion) {    
    // Prepare AttitudeSetpoint msg;
    attitude_setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    Eigen::Quaterniond rotated_quat;
    rotated_quat = rotateQuaternionFromToENU_NED(desired_quaternion);
    attitude_setpoint_msg.q_d[0] = rotated_quat.w();
    attitude_setpoint_msg.q_d[1] = rotated_quat.x();
    attitude_setpoint_msg.q_d[2] = rotated_quat.y();
    attitude_setpoint_msg.q_d[3] = rotated_quat.z();

    if (controller_output[3] > 0.1){
        attitude_setpoint_msg.thrust_body[0] = 0.0;
        attitude_setpoint_msg.thrust_body[1] = 0.0;
        attitude_setpoint_msg.thrust_body[2] = -controller_output[3];         // DO NOT FORGET THE MINUS SIGN (body NED frame)
    }
    else {
        attitude_setpoint_msg.thrust_body[2] = -0.1;
    }

    attitude_setpoint_publisher_->publish(attitude_setpoint_msg);
}

void ControllerNode::trajectory_setpoint_publisher(float x, float y, float z) {
    // Prepare TrajectorySetpoint msg
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.velocity = {0.0, 0.0, 0.0};
    msg.acceleration = {0.0, 0.0, 0.0};
    msg.yaw = 0.0;
    msg.yawspeed = 0.174533;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);

    // Debug message
    RCLCPP_INFO_ONCE(get_logger(),"Trajectory setpoint message published: [%f, %f, %f]", x, y, z);
}

void ControllerNode::inject_motor_failure(int motor_index) {
    if (motor_index >= 0 && motor_index < throttles.size()) {
        throttles[motor_index] = 0.0;
        RCLCPP_INFO_ONCE(this->get_logger(), "Injected motor failure: motor %d", motor_index);
    } else {
        RCLCPP_WARN(this->get_logger(), "Index out of bounds: %d", motor_index);
    }
}

void ControllerNode::update_controller_output() {

    // Set rotor speeds
    // std::vector<int> indices = {0, 1, 2, 3};
    // std::vector<float> values = {0.74, 0.74, 0.74, 0.74};
    // set_rotor_throttles(indices, values);
    // inject_motor_failure(0);

    // Publish the controller output
    if (current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        if (inject_motor_failure_ != -1){
            inject_motor_failure(inject_motor_failure_);
        }
        if (control_mode_ == 1){
            // trajectory_setpoint_publisher(0.0, 0.0, 5.0);
        }
        else if (control_mode_ == 2) {
            actuator_motor_publisher(throttles);
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}