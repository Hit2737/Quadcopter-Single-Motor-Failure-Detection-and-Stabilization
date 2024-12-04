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

#include "../include/px4_motor_control/controller.h"
#include <fstream>
#include <filesystem>
#include <iostream>

// Function to write to the log file
void write_log(const std::string &log_message, const std::string &log_file_name)
{
    std::string log_file_path = "src/px4_motor_control/logs/" + log_file_name + ".csv";

    // Ensure the directory exists
    std::filesystem::create_directories(std::filesystem::path(log_file_path).parent_path());

    std::ofstream log_file(log_file_path, std::ios::app);
    if (!log_file.is_open())
    {
        std::cerr << "Unable to open file for writing error log" << std::endl;
        return;
    }

    log_file << log_message << std::endl;
    log_file.close();
}

controller::controller()
{
}

void controller::compute_thrust_and_torque(
    Eigen::VectorXd *wrench, Eigen::Quaterniond *desired_quaternion)
{
    assert(wrench);

    wrench->resize(4);

    // Geometric controller based on:
    // T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3),
    // " 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, USA, 2010.

    // Trajectory tracking.
    double thrust;
    Eigen::Matrix3d R_d_w;

    // Compute translational tracking errors.
    const Eigen::Vector3d e_p =
        position_W_ - r_position_W_;

    const Eigen::Vector3d e_v =
        velocity_W_ - r_velocity_W_;

    // Store the error through x, y, and z into a .csv file
    // std::string log_file_name = "position_error_NC_1";
    // std::string log_message = std::to_string(e_p(0)) + "," + std::to_string(e_p(1)) + "," + std::to_string(e_p(2));
    // write_log(log_message, log_file_name);

    const Eigen::Vector3d I_a_d = -position_gain_.cwiseProduct(e_p) - velocity_gain_.cwiseProduct(e_v) + _uav_mass * _gravity * Eigen::Vector3d::UnitZ() + _uav_mass * r_acceleration_W_;
    thrust = I_a_d.dot(R_B_W_.col(2));
    Eigen::Vector3d B_z_d;
    B_z_d = I_a_d;
    B_z_d.normalize();

    // Calculate Desired Rotational Matrix
    const Eigen::Vector3d B_x_d(std::cos(r_yaw), std::sin(r_yaw), 0.0);
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
    B_y_d.normalize();
    R_d_w.col(0) = B_y_d.cross(B_z_d);
    R_d_w.col(1) = B_y_d;
    R_d_w.col(2) = B_z_d;

    Eigen::Quaterniond q_temp(R_d_w);
    *desired_quaternion = q_temp;

    // Attitude tracking.
    Eigen::Vector3d tau;

    const Eigen::Matrix3d e_R_matrix =
        0.5 * (R_d_w.transpose() * R_B_W_ - R_B_W_.transpose() * R_d_w);
    Eigen::Vector3d e_R;
    e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);

    // Store the error through roll, pitch, and yaw into a .csv file
    // log_file_name = "attitude_error_NC_1";
    // log_message = std::to_string(e_R(0)) + "," + std::to_string(e_R(1)) + "," + std::to_string(e_R(2));
    // write_log(log_message, log_file_name);

    // Compute the control torque
    const Eigen::Vector3d omega_ref =
        r_yaw_rate * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d e_omega = angular_velocity_B_ - R_B_W_.transpose() * R_d_w * omega_ref;
    tau = -attitude_gain_.cwiseProduct(e_R) - angular_rate_gain_.cwiseProduct(e_omega) + angular_velocity_B_.cross(_inertia_matrix.asDiagonal() * angular_velocity_B_);

    // Output the wrench
    *wrench << tau, thrust;
}
