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

#ifndef CONTROLLER_CONTROLLER_NODE_H
#define CONTROLLER_CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/int32.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <string>
#include <atomic>
#include "px4_motor_control/controller.h"

#include <chrono>
using namespace std::chrono_literals;

using std::placeholders::_1;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();
    // virtual ~controller_node();
    void update_control_loop();

private:
    controller controller_;

    // Timers
    rclcpp::TimerBase::SharedPtr controllerTimer;
    rclcpp::TimerBase::SharedPtr offboardTimer;

    // subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr failure_detection_sub_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Services
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Messages
    px4_msgs::msg::VehicleAttitudeSetpoint attitude_setpoint_msg;
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;

    // Topics namescontroller_
    std::string command_pose_topic_;
    std::string command_traj_topic_;
    std::string odometry_topic_;
    std::string status_topic_;
    std::string battery_status_topic_;
    std::string actuator_status_topic;
    std::string offboard_control_topic_;
    std::string vehicle_command_topic_;
    std::string attitude_setpoint_topic_;
    std::string thrust_setpoint_topic_;
    std::string torque_setpoint_topic_;
    std::string actuator_control_topic_;
    std::string failure_detection_topic_;
    std::atomic<int> failed_motor_{0};

    // UAV Parameters
    double _arm_length;
    double _moment_constant;
    double _thrust_constant;
    double _max_rotor_speed;
    Eigen::Vector3d _omega_to_pwm_coefficients;
    int _PWM_MIN;
    int _PWM_MAX;
    int _input_scaling;
    int _zero_position_armed;
    Eigen::MatrixXd torques_and_thrust_to_rotor_velocities_;
    Eigen::MatrixXd torques_and_thrust_to_rotor_velocities_updated_;
    Eigen::MatrixXd throttles_to_normalized_torques_and_thrust_;

    // Controller gains
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d ang_vel_gain_;

    px4_msgs::msg::VehicleStatus current_status_;
    bool connected_ = false;

    // void secureConnection();
    void arm();
    void disarm();

    // CallBacks
    void command_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg);

    void actuator_motors_publisher(const Eigen::VectorXd &throttles);
    void offboard_control_mode_publisher();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void update_allocation_matrix(int failed_motor_);
    void failure_detection_callback(const std_msgs::msg::Int32::SharedPtr fail_msg);
    void px4_inverse_not_failed(Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench);
    void px4_inverse_failed(Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench);

    inline Eigen::Vector3d rotateVectorFromToENU_NED(const Eigen::Vector3d &vec_in)
    {
        // NED (X North, Y East, Z Down) & ENU (X East, Y North, Z Up)
        Eigen::Vector3d vec_out;
        vec_out << vec_in[1], vec_in[0], -vec_in[2];
        return vec_out;
    }

    inline Eigen::Vector3d rotateVectorFromToFRD_FLU(const Eigen::Vector3d &vec_in)
    {
        // FRD (X Forward, Y Right, Z Down) & FLU (X Forward, Y Left, Z Up)
        Eigen::Vector3d vec_out;
        vec_out << vec_in[0], -vec_in[1], -vec_in[2];
        return vec_out;
    }

    inline Eigen::Quaterniond rotateQuaternionFromToENU_NED(const Eigen::Quaterniond &quat_in)
    {
        // Transform from orientation represented in ROS format to PX4 format and back
        //  * Two steps conversion:
        //  * 1. aircraft to NED is converted to aircraft to ENU (NED_to_ENU conversion)
        //  * 2. aircraft to ENU is converted to baselink to ENU (baselink_to_aircraft conversion)
        // OR
        //  * 1. baselink to ENU is converted to baselink to NED (ENU_to_NED conversion)
        //  * 2. baselink to NED is converted to aircraft to NED (aircraft_to_baselink conversion
        // NED_ENU_Q Static quaternion needed for rotating between ENU and NED frames
        Eigen::Vector3d euler_1(M_PI, 0.0, M_PI_2);
        Eigen::Quaterniond NED_ENU_Q(Eigen::AngleAxisd(euler_1.z(), Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(euler_1.y(), Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(euler_1.x(), Eigen::Vector3d::UnitX()));

        // AIRCRAFT_BASELINK_Q Static quaternion needed for rotating between aircraft and base_link frames
        Eigen::Vector3d euler_2(M_PI, 0.0, 0.0);
        Eigen::Quaterniond AIRCRAFT_BASELINK_Q(Eigen::AngleAxisd(euler_2.z(), Eigen::Vector3d::UnitZ()) *
                                               Eigen::AngleAxisd(euler_2.y(), Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(euler_2.x(), Eigen::Vector3d::UnitX()));

        return (NED_ENU_Q * quat_in) * AIRCRAFT_BASELINK_Q;
    }

    inline void eigenOdometryFromPX4Msg(const px4_msgs::msg::VehicleOdometry::SharedPtr msg,
                                        Eigen::Vector3d &position_W, Eigen::Quaterniond &orientation_B_W,
                                        Eigen::Vector3d &velocity_B, Eigen::Vector3d &angular_velocity_B)
    {

        position_W = rotateVectorFromToENU_NED(Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]));

        Eigen::Quaterniond quaternion(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        orientation_B_W = rotateQuaternionFromToENU_NED(quaternion);

        velocity_B = rotateVectorFromToENU_NED(Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]));

        angular_velocity_B = rotateVectorFromToFRD_FLU(Eigen::Vector3d(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]));
    }

    inline void eigenTrajectoryPointFromMsg(
        const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr &msg,
        Eigen::Vector3d &position_W, Eigen::Quaterniond &orientation_W_B,
        Eigen::Vector3d &velocity_W, Eigen::Vector3d &angular_velocity_W,
        Eigen::Vector3d &acceleration_W)
    {

        if (msg->transforms.empty())
        {
            return;
        }

        position_W << msg->transforms[0].translation.x,
            msg->transforms[0].translation.y,
            msg->transforms[0].translation.z;
        Eigen::Quaterniond quaternion(msg->transforms[0].rotation.w,
                                      msg->transforms[0].rotation.x,
                                      msg->transforms[0].rotation.y,
                                      msg->transforms[0].rotation.z);
        orientation_W_B = quaternion;
        if (msg->velocities.size() > 0)
        {
            velocity_W << msg->velocities[0].linear.x,
                msg->velocities[0].linear.y,
                msg->velocities[0].linear.z;
            angular_velocity_W << msg->velocities[0].angular.x,
                msg->velocities[0].angular.y,
                msg->velocities[0].angular.z;
        }
        else
        {
            velocity_W.setZero();
            angular_velocity_W.setZero();
        }
        if (msg->accelerations.size() > 0)
        {
            acceleration_W << msg->accelerations[0].linear.x,
                msg->accelerations[0].linear.y,
                msg->accelerations[0].linear.z;
        }
        else
        {
            acceleration_W.setZero();
        }
    }

    inline void eigenTrajectoryPointFromPoseMsg(
        const geometry_msgs::msg::PoseStamped::SharedPtr &msg, Eigen::Vector3d &position_W, Eigen::Quaterniond &orientation_W_B)
    {

        position_W << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        Eigen::Quaterniond quaternion(msg->pose.orientation.w,
                                      msg->pose.orientation.x,
                                      msg->pose.orientation.y,
                                      msg->pose.orientation.z);
        orientation_W_B = quaternion;
    }

    void load_setup_parameters_and_matrices()
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
        this->declare_parameter("topics_names.failure_detection_topic", "default");

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
        failure_detection_topic_ = this->get_parameter("topics_names.failure_detection_topic").as_string();
        failed_motor_.store(this->get_parameter("failed_motor").as_int());

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

        // Compute Control Allocation and Actuator Effect matrices
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

        torques_and_thrust_to_rotor_velocities_.setZero();
        torques_and_thrust_to_rotor_velocities_ =
            rotor_velocities_to_torques_and_thrust.completeOrthogonalDecomposition().pseudoInverse();
    }
};

#endif
