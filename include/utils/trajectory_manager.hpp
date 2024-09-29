#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

template <int _Dof>
class TrajectoryManager
{
public:
    TrajectoryManager() = delete;
    ~TrajectoryManager() = default;
    TrajectoryManager(Eigen::Vector<double, _Dof> current_q, std::vector<Eigen::Vector<double, _Dof>> waypoints, std::vector<double> time_steps);
    TrajectoryManager(Eigen::Vector<double, _Dof> current_q, std::vector<Eigen::Vector<double, _Dof>> waypoints, std::vector<Eigen::Vector<double, _Dof>> waypoints_vel, std::vector<double> time_steps);

    Eigen::Matrix<double, 2, _Dof> getTrajectory(double const& time);

private:
    std::vector<double> time_steps_;
    std::vector<std::vector<Eigen::Vector<double, 4>>> coeff_q_;
};

#include "trajectory_manager.tpp"

#endif