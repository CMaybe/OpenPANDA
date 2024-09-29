#include "utils/trajectory_manager.hpp"

template <int _Dof>
TrajectoryManager<_Dof>::TrajectoryManager(Eigen::Vector<double, _Dof> current_q,
                                           std::vector<Eigen::Vector<double, _Dof>> waypoints,
                                           std::vector<double> time_steps) : time_steps_(time_steps)
{
    assert(waypoints.size() == time_steps.size() && "waypoints and time_steps must be same size");
    std::vector<std::vector<Eigen::Vector<double, 4>>> q(waypoints.size() - 1);
    waypoints.insert(waypoints.begin(), current_q);
    time_steps_.insert(time_steps_.begin(), 0.0);
    coeff_q_.resize(waypoints.size() - 1);
    Eigen::Matrix<double, 4, 4> polynomial_matrix;

    for (int i = 0; i < waypoints.size() - 1; i++)
    {
        for (int j = 0; j < _Dof; j++)
        {
            q[i][j](0) = waypoints[i](j);
            q[i][j](1) = 0.0;
            q[i][j](2) = waypoints[i + 1](j);
            q[i][j](3) = 0.0;
        }
    }
    for (int i = 0; i < coeff_q_.size(); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            polynomial_matrix(0, j) = pow(time_steps_[i], j);
            if (j > 0)
                polynomial_matrix(1, j) = j * pow(time_steps_[i], j - 1);
            polynomial_matrix(2, j) = pow(time_steps_[i + 1], i);
            if (j > 0)
                polynomial_matrix(3, j) = j * pow(time_steps_[i + 1], j - 1);
        }
        for (int j = 0; j < _Dof; j++)
        {
            polynomial_matrix.setIdentity();
            coeff_q_[i][j] = polynomial_matrix.inverse() * q[i][j];
        }
    }
}

template <int _Dof>
TrajectoryManager<_Dof>::TrajectoryManager(Eigen::Vector<double, _Dof> current_q,
                                           std::vector<Eigen::Vector<double, _Dof>> waypoints,
                                           std::vector<Eigen::Vector<double, _Dof>> waypoints_vel,
                                           std::vector<double> time_steps) : time_steps_(time_steps)
{
    assert(waypoints.size() == time_steps.size() && "waypoints and time_steps must be same size");

    std::vector<std::vector<Eigen::Vector<double, 4>>> q(waypoints.size() - 1);
    waypoints.insert(waypoints.begin(), current_q);
    waypoints_vel.insert(waypoints_vel.begin(), 0.0);
    time_steps_.insert(time_steps_.begin(), 0.0);
    coeff_q_.resize(waypoints.size() - 1);
    Eigen::Matrix<double, 4, 4> polynomial_matrix;

    for (int i = 0; i < waypoints.size() - 1; i++)
    {
        for (int j = 0; j < _Dof; j++)
        {
            q[i][j](0) = waypoints[i](j);
            q[i][j](1) = waypoints_vel[i](j);
            q[i][j](2) = waypoints[i + 1](j);
            q[i][j](3) = waypoints_vel[i + 1](j);
        }
    }
    for (int i = 0; i < coeff_q_.size(); i++)
    {
        coeff_q_[i].resize(_Dof);
        for (int j = 0; j < 4; j++)
        {
            polynomial_matrix(0, j) = pow(time_steps_[i], j);
            if (j > 0)
                polynomial_matrix(1, j) = j * pow(time_steps_[i], j - 1);
            polynomial_matrix(2, j) = pow(time_steps_[i + 1], i);
            if (j > 0)
                polynomial_matrix(3, j) = j * pow(time_steps_[i + 1], j - 1);
        }
        for (int j = 0; j < _Dof; j++)
        {
            polynomial_matrix.setIdentity();
            coeff_q_[i][j] = polynomial_matrix.inverse() * q[i][j];
        }
    }
}

template <int _Dof>
Eigen::Matrix<double, 2, _Dof> TrajectoryManager<_Dof>::getTrajectory(double const& time)
{
    Eigen::Matrix<double, 2, _Dof> result;
    Eigen::Matrix<double, 2, 4> time_pow;
    int idx;
    time_pow.setZero();
    for (int i = 0; i < 4; ++i)
    {
        time_pow(0, i) = pow(time, i);
        time_pow(1, i) = i * pow(time, i - 1);
    }
    for (int i = 0; i < time_steps_.size(); i++)
    {
        if (time > time_steps_[i])
        {
            idx = i - 1;
            break;
        }
    }

    for (int i = 0; i < _Dof; i++)
    {
        result.col(i) = time_pow * coeff_q_[idx][i];
    }
    return result;
}