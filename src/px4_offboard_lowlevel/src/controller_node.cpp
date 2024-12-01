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
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

std::atomic<int> failed_motor_{0};

ControllerNode::ControllerNode()
    : Node("controller_node")
{
    loadParams();
    RCLCPP_INFO(this->get_logger(), "Controller Node has been started");
    compute_ControlAllocation_and_ActuatorEffect_matrices();

    // Defining the compatible ROS 2 predefined QoS for PX4 topics
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Subscribers
    vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(odometry_topic_, qos, std::bind(&ControllerNode::vehicle_odometryCallback, this, _1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(status_topic_, qos, std::bind(&ControllerNode::vehicleStatusCallback, this, _1));
    command_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(command_pose_topic_, 10, std::bind(&ControllerNode::commandPoseCallback, this, _1));
    // failure_detection_sub_ = this->create_subscription<std_msgs::msg>
    // Publishers
    actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(actuator_control_topic_, 10);
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_control_topic_, 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic_, 10);

    // Parameters subscriber
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ControllerNode::parametersCallback, this, std::placeholders::_1));

    // Timers
    std::chrono::duration<double> offboard_period(0.33);
    std::chrono::duration<double> controller_period(0.01);
    offboardTimer = this->create_wall_timer(offboard_period, [=]()
                                            { publishOffboardControlModeMsg(); });
    controllerTimer = this->create_wall_timer(controller_period, [=]()
                                              { updateControllerOutput(); });
}

rcl_interfaces::msg::SetParametersResult ControllerNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "failed_motor")
        {
            int new_value = param.as_int();
            failed_motor_.store(new_value);
            RCLCPP_INFO(this->get_logger(), "Updated failed_motor to [%d]", new_value);

            UpdateAllocationMatrix(new_value); // ********************************** Here we have to change the new_value to the detected failed motor from the detector
        }
    }
    return result;
}

void ControllerNode::loadParams()
{
    // UAV Parameters
    this->declare_parameter("failed_motor", 0);
    this->declare_parameter("uav_parameters.mass", 0.0);
    this->declare_parameter("uav_parameters.arm_length", 0.0);
    this->declare_parameter("uav_parameters.num_of_arms", 4);
    this->declare_parameter("uav_parameters.moment_constant", 0.0);
    this->declare_parameter("uav_parameters.thrust_constant", 0.0);
    this->declare_parameter("uav_parameters.max_rotor_speed", 0);
    this->declare_parameter("uav_parameters.gravity", 0.0);
    this->declare_parameter("uav_parameters.PWM_MIN", 0);
    this->declare_parameter("uav_parameters.PWM_MAX", 0);
    this->declare_parameter("uav_parameters.input_scaling", 0);
    this->declare_parameter("uav_parameters.zero_position_armed", 0);
    this->declare_parameter("uav_parameters.inertia.x", 0.0);
    this->declare_parameter("uav_parameters.inertia.y", 0.0);
    this->declare_parameter("uav_parameters.inertia.z", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_2", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_1", 0.0);
    this->declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_0", 0.0);

    double _uav_mass = this->get_parameter("uav_parameters.mass").as_double();
    _arm_length = this->get_parameter("uav_parameters.arm_length").as_double();
    _num_of_arms = this->get_parameter("uav_parameters.num_of_arms").as_int();
    _moment_constant = this->get_parameter("uav_parameters.moment_constant").as_double();
    _thrust_constant = this->get_parameter("uav_parameters.thrust_constant").as_double();
    _max_rotor_speed = this->get_parameter("uav_parameters.max_rotor_speed").as_int();
    double _gravity = this->get_parameter("uav_parameters.gravity").as_double();
    _PWM_MIN = this->get_parameter("uav_parameters.PWM_MIN").as_int();
    _PWM_MAX = this->get_parameter("uav_parameters.PWM_MAX").as_int();
    _input_scaling = this->get_parameter("uav_parameters.input_scaling").as_int();
    _zero_position_armed = this->get_parameter("uav_parameters.zero_position_armed").as_int();
    double _inertia_x = this->get_parameter("uav_parameters.inertia.x").as_double();
    double _inertia_y = this->get_parameter("uav_parameters.inertia.y").as_double();
    double _inertia_z = this->get_parameter("uav_parameters.inertia.z").as_double();
    double _omega_to_pwm_coefficient_x_2 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_2").as_double();
    double _omega_to_pwm_coefficient_x_1 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_1").as_double();
    double _omega_to_pwm_coefficient_x_0 = this->get_parameter("uav_parameters.omega_to_pwm_coefficient.x_0").as_double();
    Eigen::Vector3d _inertia_matrix;
    _inertia_matrix << _inertia_x, _inertia_y, _inertia_z;
    _omega_to_pwm_coefficients << _omega_to_pwm_coefficient_x_2, _omega_to_pwm_coefficient_x_1, _omega_to_pwm_coefficient_x_0;

    // Topics Names
    this->declare_parameter("topics_names.command_pose_topic", "default");
    this->declare_parameter("topics_names.command_traj_topic", "default");
    this->declare_parameter("topics_names.odometry_topic", "default");
    this->declare_parameter("topics_names.status_topic", "default");
    this->declare_parameter("topics_names.battery_status_topic", "default");
    this->declare_parameter("topics_names.actuator_status_topic", "default");
    this->declare_parameter("topics_names.offboard_control_topic", "default");
    this->declare_parameter("topics_names.vehicle_command_topic", "default");
    this->declare_parameter("topics_names.attitude_setpoint_topic", "default");
    this->declare_parameter("topics_names.thrust_setpoints_topic", "default");
    this->declare_parameter("topics_names.torque_setpoints_topic", "default");
    this->declare_parameter("topics_names.actuator_control_topic", "default");

    command_pose_topic_ = this->get_parameter("topics_names.command_pose_topic").as_string();
    command_traj_topic_ = this->get_parameter("topics_names.command_traj_topic").as_string();
    odometry_topic_ = this->get_parameter("topics_names.odometry_topic").as_string();
    status_topic_ = this->get_parameter("topics_names.status_topic").as_string();
    battery_status_topic_ = this->get_parameter("topics_names.battery_status_topic").as_string();
    actuator_status_topic = this->get_parameter("topics_names.actuator_status_topic").as_string();
    offboard_control_topic_ = this->get_parameter("topics_names.offboard_control_topic").as_string();
    vehicle_command_topic_ = this->get_parameter("topics_names.vehicle_command_topic").as_string();
    attitude_setpoint_topic_ = this->get_parameter("topics_names.attitude_setpoint_topic").as_string();
    thrust_setpoint_topic_ = this->get_parameter("topics_names.thrust_setpoints_topic").as_string();
    torque_setpoint_topic_ = this->get_parameter("topics_names.torque_setpoints_topic").as_string();
    actuator_control_topic_ = this->get_parameter("topics_names.actuator_control_topic").as_string();
    failed_motor_.store(this->get_parameter("failed_motor").as_int());

    // Load logic switches
    this->declare_parameter("sitl_mode", true);
    this->declare_parameter("control_mode", 1);

    in_sitl_mode_ = this->get_parameter("sitl_mode").as_bool();
    control_mode_ = this->get_parameter("control_mode").as_int();

    // Controller gains
    this->declare_parameter("control_gains.K_p_x", 0.0);
    this->declare_parameter("control_gains.K_p_y", 0.0);
    this->declare_parameter("control_gains.K_p_z", 0.0);
    this->declare_parameter("control_gains.K_v_x", 0.0);
    this->declare_parameter("control_gains.K_v_y", 0.0);
    this->declare_parameter("control_gains.K_v_z", 0.0);
    this->declare_parameter("control_gains.K_R_x", 0.0);
    this->declare_parameter("control_gains.K_R_y", 0.0);
    this->declare_parameter("control_gains.K_R_z", 0.0);
    this->declare_parameter("control_gains.K_w_x", 0.0);
    this->declare_parameter("control_gains.K_w_y", 0.0);
    this->declare_parameter("control_gains.K_w_z", 0.0);

    position_gain_ << this->get_parameter("control_gains.K_p_x").as_double(),
        this->get_parameter("control_gains.K_p_y").as_double(),
        this->get_parameter("control_gains.K_p_z").as_double();

    velocity_gain_ << this->get_parameter("control_gains.K_v_x").as_double(),
        this->get_parameter("control_gains.K_v_y").as_double(),
        this->get_parameter("control_gains.K_v_z").as_double();

    attitude_gain_ << this->get_parameter("control_gains.K_R_x").as_double(),
        this->get_parameter("control_gains.K_R_y").as_double(),
        this->get_parameter("control_gains.K_R_z").as_double();

    ang_vel_gain_ << this->get_parameter("control_gains.K_w_x").as_double(),
        this->get_parameter("control_gains.K_w_y").as_double(),
        this->get_parameter("control_gains.K_w_z").as_double();

    controller_.setUavMass(_uav_mass);
    controller_.setInertiaMatrix(_inertia_matrix);
    controller_.setGravity(_gravity);
    controller_.setKPositionGain(position_gain_);
    controller_.setKVelocityGain(velocity_gain_);
    controller_.setKAttitudeGain(attitude_gain_);
    controller_.setKAngularRateGain(ang_vel_gain_);
}

void ControllerNode::compute_ControlAllocation_and_ActuatorEffect_matrices()
{
    const double kDegToRad = M_PI / 180.0;
    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust;
    Eigen::MatrixXd mixing_matrix;

    if (_num_of_arms == 4)
    {
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
    }
    else
    {
        std::cout << ("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n");
    }

    Eigen::Vector4d k;
    k << _thrust_constant * _arm_length,
        _thrust_constant * _arm_length,
        _moment_constant * _thrust_constant,
        _thrust_constant;

    rotor_velocities_to_torques_and_thrust = k.asDiagonal() * rotor_velocities_to_torques_and_thrust;

    std::cout << "rotor_velocities_to_torques_and_thrust = " << rotor_velocities_to_torques_and_thrust << std::endl;

    torques_and_thrust_to_rotor_velocities_.setZero();
    torques_and_thrust_to_rotor_velocities_ =
        rotor_velocities_to_torques_and_thrust.completeOrthogonalDecomposition().pseudoInverse();

    std::cout << "rotor_velocities_to_torques_and_thrust = " << rotor_velocities_to_torques_and_thrust << std::endl;
    std::cout << "torques_and_thrust_to_rotor_velocities = " << torques_and_thrust_to_rotor_velocities_ << std::endl;
    std::cout << "throttles_to_normalized_torques_and_thrust_ = " << throttles_to_normalized_torques_and_thrust_ << std::endl;
}

void ControllerNode::UpdateAllocationMatrix(int failed_motor_)
{
    const double kDegToRad = M_PI / 180.0;
    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust;
    Eigen::MatrixXd mixing_matrix;
    if (_num_of_arms == 4)
    {
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
    }
    else
    {
        std::cout << ("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n");
    }

    Eigen::Vector4d k;
    k << _thrust_constant * _arm_length,
        _thrust_constant * _arm_length,
        _moment_constant * _thrust_constant,
        _thrust_constant;
    rotor_velocities_to_torques_and_thrust = k.asDiagonal() * rotor_velocities_to_torques_and_thrust;

    Eigen::MatrixXd rotor_velocities_to_torques_and_thrust_updated(3, 3);

    int update_i = 0;
    for (int i = 0; i < 4; i++)
    {
        if (i == 2)
            continue;

        int update_j = 0;
        for (int j = 0; j < 4; j++)
        {
            if (j == failed_motor_)
                continue;

            rotor_velocities_to_torques_and_thrust_updated(update_i, update_j) = rotor_velocities_to_torques_and_thrust(i, j);
            ++update_j;
        }
        ++update_i;
    }

    std::cout << "rotor_velocities_to_torques_and_thrust_updated = " << rotor_velocities_to_torques_and_thrust_updated << std::endl;
    torques_and_thrust_to_rotor_velocities_.resize(3, 3);
    torques_and_thrust_to_rotor_velocities_ =
        rotor_velocities_to_torques_and_thrust_updated.completeOrthogonalDecomposition().pseudoInverse();
    std::cout << "torques_and_thrust_to_rotor_velocities = " << torques_and_thrust_to_rotor_velocities_ << std::endl;
}

void ControllerNode::px4InverseSITL(Eigen::Vector4d *normalized_torque_and_thrust, Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench)
{
    RCLCPP_INFO(this->get_logger(), "px4InverseSITL");
    Eigen::VectorXd omega;
    normalized_torque_and_thrust->setZero();
    Eigen::VectorXd ones_temp;
    if (_num_of_arms == 6)
    {
        omega.resize(6);
        omega.setZero();
        throttles->resize(6);
        throttles->setZero();
        ones_temp.resize(6);
        ones_temp = Eigen::VectorXd::Ones(6, 1);
    }
    else if (_num_of_arms == 4)
    {
        omega.resize(4);
        omega.setZero();
        throttles->resize(4);
        throttles->setZero();
        ones_temp.resize(4);
        ones_temp = Eigen::VectorXd::Ones(4, 1);
    }
    else if (_num_of_arms == 44)
    {
        omega.resize(8);
        omega.setZero();
        throttles->resize(8);
        throttles->setZero();
        ones_temp.resize(8);
        ones_temp = Eigen::VectorXd::Ones(8, 1);
    }
    else
    {
        std::cout << ("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n");
    }

    Eigen::VectorXd modified_wrench;
    if (failed_motor_)
    {
        modified_wrench.resize(3);
        modified_wrench << (*wrench)(0), (*wrench)(1), (*wrench)(3);

        Eigen::VectorXd omega_temp(3);
        omega.setZero();
        omega_temp = torques_and_thrust_to_rotor_velocities_ * (modified_wrench);

        for (int i = 0, update_i = 0; i < 4; i++)
        {
            if (i == failed_motor_)
                continue;
            omega(i) = omega_temp(update_i);
            update_i++;
        }
    }
    else
    {
        omega = torques_and_thrust_to_rotor_velocities_ * (*wrench);
    }

    omega = omega.cwiseSqrt();
    *throttles = (omega - (_zero_position_armed * ones_temp));
    *throttles /= (_input_scaling);
    *normalized_torque_and_thrust = throttles_to_normalized_torques_and_thrust_ * (*throttles);
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

void ControllerNode::publishOffboardControlModeMsg()
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
    RCLCPP_INFO_ONCE(get_logger(), "Offboard enabled");
}

void ControllerNode::commandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{ // When a command is received
    // initialize vectors
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    eigenTrajectoryPointFromPoseMsg(pose_msg, position, orientation);
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first command message.");
    controller_.setTrajectoryPoint(position, orientation); // Send the command to controller_ obj
}

void ControllerNode::vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
    //  Debug message
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first odometry message.");

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;

    eigenOdometryFromPX4Msg(odom_msg,
                            position, orientation, velocity, angular_velocity);

    controller_.setOdometry(position, orientation, velocity, angular_velocity);
}

void ControllerNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg)
{
    current_status_ = *status_msg;
    if (current_status_.arming_state == 2)
    {
        RCLCPP_INFO_ONCE(get_logger(), "ARMED - vehicle_status_msg.");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "NOT ARMED - vehicle_status_msg.");
        arm();
    }
    if (current_status_.nav_state == 14)
    {
        RCLCPP_INFO_ONCE(get_logger(), "OFFBOARD - vehicle_status_msg.");
    }
    else
    {
        RCLCPP_INFO(get_logger(), "NOT OFFBOARD - vehicle_status_msg.");
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    }
}

void ControllerNode::publishActuatorMotorsMsg(const Eigen::VectorXd &throttles)
{
    // Lockstep should be disabled from PX4 and from the model.sdf file
    // direct motor throttles control
    // Prepare msg
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;
    actuator_motors_msg.control = {(float)throttles[0], (float)throttles[1], (float)throttles[2], (float)throttles[3],
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")};
    actuator_motors_msg.reversible_flags = 0;
    actuator_motors_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
    actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

    actuator_motors_publisher_->publish(actuator_motors_msg);
}

void ControllerNode::updateControllerOutput()
{
    //  calculate controller output
    Eigen::VectorXd controller_output;
    Eigen::Quaterniond desired_quaternion;
    controller_.calculateControllerOutput(&controller_output, &desired_quaternion);

    // Normalize the controller output
    Eigen::Vector4d normalized_torque_thrust;
    Eigen::VectorXd throttles;
    px4InverseSITL(&normalized_torque_thrust, &throttles, &controller_output);

    // Publish the controller output
    if (current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
        publishActuatorMotorsMsg(throttles);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}