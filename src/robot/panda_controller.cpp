#include "robot/panda_controller.hpp"
#include <iostream>
#include <cmath>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/duration.h>
#include <array>

PandaController::PandaController(const std::string &robot_ip, const std::string &gripper_ip, std::mutex &mutex_panda)
    : robot_ip_(robot_ip), gripper_ip_(gripper_ip), robot_(robot_ip), mutex_panda_(mutex_panda), gripper_(gripper_ip),
      time_step_(0), trajectory_time_(21.0), running_(false)
{

    const double translational_gain{120.0};
    const double rotational_gain{10.0};
    kp_.resize(6, 6);
    kd_.resize(6, 6);
    ki_.resize(6, 6);
    error_sum_.resize(6);
    error_sum_.setZero();
    kp_.setZero();
    kp_.topLeftCorner(3, 3) << translational_gain * Eigen::MatrixXd::Identity(3, 3);
    kp_.bottomRightCorner(3, 3) << rotational_gain * Eigen::MatrixXd::Identity(3, 3);

    kd_.setZero();
    kd_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_gain) * Eigen::MatrixXd::Identity(3, 3);
    kd_.bottomRightCorner(3, 3) << 1.0 * sqrt(rotational_gain) * Eigen::MatrixXd::Identity(3, 3);

    ki_.setZero();
    ki_.topLeftCorner(3, 3) << 1 * Eigen::MatrixXd::Identity(3, 3);
    ki_.bottomRightCorner(3, 3) << 0.01 * Eigen::MatrixXd::Identity(3, 3);

    traj_point_ << 0.4, 0.4, 0.4, 0.4,
        -0.3, -0.3, 0.3, 0.3,
        0.1, 0.5, 0.5, 0.1;

    for (int i = 0; i < 8; ++i)
    {
        time_point_[i] = trajectory_time_ / 7 * i;
    }
}

Eigen::Vector<double, 7> PandaController::getRobotJointPosition() const
{
    std::lock_guard<std::mutex> lock(mutex_panda_);
    return this->q_;
}

Eigen::Matrix<double, 2, 3> PandaController::generateTrajectory(double time, Eigen::Matrix<double, 2, 3> begin, Eigen::Matrix<double, 2, 3> end, double begin_time, double end_time)
{
    Eigen::Matrix<double, 4, 1> x, y, z, coff_x, coff_y, coff_z;
    Eigen::Matrix<double, 4, 4> coeff;
    Eigen::Matrix<double, 2, 3> result;

    x.setZero();
    y.setZero();
    z.setZero();
    coeff.setIdentity();
    x.topRows(2) = begin.col(0);
    x.bottomRows(2) = end.col(0);
    y.topRows(2) = begin.col(1);
    y.bottomRows(2) = end.col(1);
    z.topRows(2) = begin.col(2);
    z.bottomRows(2) = end.col(2);

    for (int i = 0; i < 4; ++i)
    {
        coeff(0, i) = pow(begin_time, i);
        if (i > 0)
            coeff(1, i) = i * pow(begin_time, i - 1);
        coeff(2, i) = pow(end_time, i);
        if (i > 0)
            coeff(3, i) = i * pow(end_time, i - 1);
    }
    coff_x = coeff.inverse() * x;
    coff_y = coeff.inverse() * y;
    coff_z = coeff.inverse() * z;

    Eigen::Matrix<double, 2, 4> time_pow;
    time_pow.setZero();
    for (int i = 0; i < 4; ++i)
    {
        time_pow(0, i) = pow(time, i);
        time_pow(1, i) = i * pow(time, i - 1);
    }

    result.col(0) = time_pow * coff_x;
    result.col(1) = time_pow * coff_y;
    result.col(2) = time_pow * coff_z;

    return result;
}

void PandaController::controlLoop()
{
    try
    {
        robot_.setCollisionBehavior({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
                                    {{50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0}},
                                    {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
                                    {{50000.0, 50000.0, 50000.0, 50000.0, 50000.0, 50000.0}});
        robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        franka::Model model = robot_.loadModel();
        franka::RobotState initial_state = robot_.readOnce();
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d initial_position(initial_transform.translation());
        Eigen::Matrix3d rotation_d = Eigen::Matrix3d::Identity();
        rotation_d(1, 1) = -1;
        rotation_d(2, 2) = -1;

        auto force_control_callback = [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques
        {
            if (running_ == false)
            {
                throw std::runtime_error("stop robot");
            }
            std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            std::array<double, 49> mass_array = model.mass(robot_state);
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            std::array<double, 7> gravity_array = model.gravity(robot_state);

            Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jacobian_array.data());  // J matrix
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());   // Joint q
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data()); // Joint qdot
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Matrix3d rotation(transform.rotation());

            Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(mass_array.data()); // Mass matrix
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_(robot_state.tau_J.data()); // Joint torque

            Eigen::Matrix<double, 3, 7> Jv = J.topRows(3);
            Eigen::Matrix<double, 3, 7> Jw = J.bottomRows(3);
            Eigen::Matrix<double, 6, 6> lambda_inv = J * mass_matrix.inverse() * J.transpose();
            Eigen::Matrix<double, 6, 6> lambda = lambda_inv.inverse();

            Eigen::Matrix<double, 3, 3> lambda_v_inv_ = Jv * mass_matrix.inverse() * Jv.transpose();
            Eigen::Matrix<double, 3, 3> lambda_v_ = lambda_v_inv_.inverse();

            Eigen::Matrix<double, 3, 3> lambda_w_inv_ = Jw * mass_matrix.inverse() * Jw.transpose();
            Eigen::Matrix<double, 3, 3> lambda_w_ = lambda_w_inv_.inverse();

            Eigen::VectorXd desired_force_torque(6), tau_cmd(7);
            desired_force_torque.setZero();

            {
                std::lock_guard<std::mutex> lock(mutex_panda_);
                this->q_ = q;
            }

            Eigen::Matrix<double, 7, 7> I7;
            I7.setIdentity();
            Eigen::MatrixXd J_T = J.transpose();
            Eigen::MatrixXd J_pinv = (J_T * J).inverse() * J_T;
            Eigen::MatrixXd Null_projector = (I7 - J_pinv * J);
            Eigen::VectorXd q_des_(7);
            Eigen::VectorXd q_dot_des_(7);

            Eigen::Vector3d position_d(0, 0, 0);
            Eigen::Vector3d velocity_d(0, 0, 0);

            Eigen::Matrix<double, 6, 1> error;
            Eigen::VectorXd tau_task(7), tau_d(7), tau_null(7);
            Eigen::Matrix<double, 2, 3> traj_result;

            Eigen::Matrix<double, 2, 3> begin, end;
            begin.setZero();
            end.setZero();

            if (time_step_ == 0)
            {
                time_step_ += period.toSec();
            }

            if (time_step_ > 0)
            {
                time_step_ += period.toSec();
                if (time_step_ < time_point_[1])
                {
                    begin.row(0) = initial_position.transpose();
                    end.row(0) = traj_point_.col(0).transpose();
                    traj_result = generateTrajectory(time_step_, begin, end, 0, time_point_[1]);
                    position_d = traj_result.row(0).transpose();
                    velocity_d = traj_result.row(1).transpose();
                }
                else if (time_step_ < time_point_[2])
                {
                    begin.row(0) = traj_point_.col(0).transpose();
                    end.row(0) = traj_point_.col(1).transpose();
                    traj_result = generateTrajectory(time_step_, begin, end, time_point_[1], time_point_[2]);
                    position_d = traj_result.row(0).transpose();
                    velocity_d = traj_result.row(1).transpose();
                }
                else if (time_step_ < time_point_[3])
                {
                    begin.row(0) = traj_point_.col(1).transpose();
                    end.row(0) = traj_point_.col(2).transpose();
                    traj_result = generateTrajectory(time_step_, begin, end, time_point_[2], time_point_[3]);
                    position_d = traj_result.row(0).transpose();
                    velocity_d = traj_result.row(1).transpose();
                }
                else if (time_step_ < time_point_[4])
                {
                    begin.row(0) = traj_point_.col(2).transpose();
                    end.row(0) = traj_point_.col(3).transpose();
                    traj_result = generateTrajectory(time_step_, begin, end, time_point_[3], time_point_[4]);
                    position_d = traj_result.row(0).transpose();
                    velocity_d = traj_result.row(1).transpose();
                }
                else if (time_step_ < time_point_[5])
                {
                    begin.row(0) = traj_point_.col(3).transpose();
                    end.row(0) = traj_point_.col(2).transpose();
                    traj_result = generateTrajectory(time_step_, begin, end, time_point_[4], time_point_[5]);
                    position_d = traj_result.row(0).transpose();
                    velocity_d = traj_result.row(1).transpose();
                }
                else if (time_step_ < time_point_[6])
                {
                    begin.row(0) = traj_point_.col(2).transpose();
                    end.row(0) = traj_point_.col(1).transpose();
                    traj_result = generateTrajectory(time_step_, begin, end, time_point_[5], time_point_[6]);
                    position_d = traj_result.row(0).transpose();
                    velocity_d = traj_result.row(1).transpose();
                }
                else if (time_step_ < time_point_[7])
                {
                    begin.row(0) = traj_point_.col(1).transpose();
                    end.row(0) = traj_point_.col(0).transpose();
                    traj_result = generateTrajectory(time_step_, begin, end, time_point_[6], time_point_[7]);
                    position_d = traj_result.row(0).transpose();
                    velocity_d = traj_result.row(1).transpose();
                }
                else if (time_step_ > trajectory_time_)
                {
                    time_step_ = time_point_[1];
                }
            }

            error.head(3) << position_d - position;
            error.tail(3) = (rotation.col(0).cross(rotation_d.col(0)) + rotation.col(1).cross(rotation_d.col(1)) + rotation.col(2).cross(rotation_d.col(2)));

            error_sum_ += error * 0.02;
            desired_force_torque.head(3) = (kp_.topLeftCorner(3, 3) * error.head(3) + kd_.topLeftCorner(3, 3) * (velocity_d - Jv * dq)) + ki_.topLeftCorner(3, 3) * error_sum_.head(3);
            desired_force_torque.tail(3) = (kp_.bottomRightCorner(3, 3) * error.tail(3) - kd_.bottomRightCorner(3, 3) * (Jw * dq)) + ki_.bottomRightCorner(3, 3) * error_sum_.tail(3);
            tau_null = 2.0 * (-q) + 0.5 * (-dq);
            tau_task << Jv.transpose() * desired_force_torque.head(3) + Jw.transpose() * desired_force_torque.tail(3) + Null_projector * tau_null;

            tau_cmd = tau_task;

            tau_cmd.setZero();
            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;
            return tau_d_array;
        };

        robot_.control(force_control_callback);
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
    }
}

void PandaController::start()
{
    running_ = true;
    control_thread_ = std::thread(&PandaController::controlLoop, this);
}

void PandaController::stop()
{
    if (running_)
    {
        running_ = false;
        if (control_thread_.joinable())
        {
            control_thread_.join();
        }
    }
}
