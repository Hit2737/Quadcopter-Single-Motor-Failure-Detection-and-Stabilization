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

#include "../include/px4_offboard_lowlevel/controller.h"

controller::controller()
{
}

void controller::compute_thrust_and_torque(
    Eigen::VectorXd *wrench, Eigen::Vector3d *v_in, Eigen::Vector3d *y_, Eigen::Quaterniond *desired_quaternion, double time_step)
{
    assert(wrench);

    wrench->resize(4);

    // Geometric controller based on:
    // T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3),
    // " 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, USA, 2010.

    // Trajectory tracking.
    double thrust;
    Eigen::Matrix3d R_d_w;
    R_d_w.setZero();

    // Compute translational tracking errors.
    const Eigen::Vector3d e_p =
        position_W_ - r_position_W_;

    const Eigen::Vector3d e_v =
        velocity_W_ - r_velocity_W_;

    const Eigen::Vector3d I_a_d = -position_gain_.cwiseProduct(e_p) - velocity_gain_.cwiseProduct(e_v) + _uav_mass * _gravity * Eigen::Vector3d::UnitZ() + _uav_mass * r_acceleration_W_;
    thrust = I_a_d.dot(R_B_W_.col(2));
    Eigen::Vector3d B_z_d;
    B_z_d = I_a_d;
    B_z_d.normalize();

    // New Approach
    const Eigen::Vector3d n_I_dot_des = (I_a_d - I_a_d_prev) / time_step;
    const Eigen::Vector3d attitude_rate_B_ = (attitude_B_ - attitude_B_prev) / time_step;
    const Eigen::Vector3d n_B = R_B_W_.col(2);
    const Eigen::Vector3d n_B_des = R_B_W_ * B_z_d;
    // const Eigen::Vector3d n_B_dot_des = n_B_des.cross(attitude_rate_B_) + R_B_W_ * n_I_dot_des;
    const double h1 = n_B_des(0), h2 = n_B_des(1), h3 = n_B_des(2);
    const Eigen::Vector2d v_out = {kx * (n_B(0) - h1), ky * (n_B(1) - h2)};

    // Calculate Desired p, q
    Eigen::Matrix2d transform_matrix;
    transform_matrix << 0, 1.0 / h3,
        -1.0 / h3, 0;

    Eigen::Vector2d intermediate = v_out - Eigen::Vector2d(h2 * attitude_rate_B_(2), -h1 * attitude_rate_B_(2)) - Eigen::Vector2d(n_I_dot_des(0), n_I_dot_des(1));
    Eigen::Vector2d p_q_des = transform_matrix * intermediate;

    // Calculate Desired f_B_z_des
    Eigen::Vector3d something = R_B_W_ * (I_a_d - _uav_mass * _gravity * Eigen::Vector3d::UnitZ());
    double f_B_z_des = (something.dot(n_B_des)) / n_B(2);

    // Calculate V_in
    Eigen::Vector2d p_q_dot_des = (p_q_des - p_q_des_prev) / time_step;
    integral_f_B_z_des += (f_B_z_des - _uav_mass * acceleration_B_(2)) * time_step;
    *v_in << p_q_dot_des(0) + k1 * (p_q_des(0) - attitude_B_(0)), p_q_dot_des(1) + k2 * (p_q_des(1) - attitude_B_(1)), f_B_z_des + k3 * (integral_f_B_z_des);

    // Calculate y_
    integral_f_B_z += _uav_mass * acceleration_B_(2) * time_step;
    *y_ << attitude_rate_B_(0), attitude_rate_B_(1), integral_f_B_z;

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
    const Eigen::Vector3d omega_ref =
        r_yaw_rate * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d e_omega = angular_velocity_B_ - R_B_W_.transpose() * R_d_w * omega_ref;
    tau = -attitude_gain_.cwiseProduct(e_R) - angular_rate_gain_.cwiseProduct(e_omega) + angular_velocity_B_.cross(_inertia_matrix.asDiagonal() * angular_velocity_B_);

    // prev updates
    I_a_d_prev = I_a_d;
    attitude_B_prev = attitude_B_;
    p_q_des_prev = p_q_des;

    // Output the wrench
    *wrench << tau, thrust;
}
