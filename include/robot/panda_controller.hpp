#ifndef PANDA_CONTROLLER_HPP
#define PANDA_CONTROLLER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Jacobi>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <array>
#include <functional>
#include <thread>
#include <mutex>

class PandaController
{
public:
    PandaController(const std::string &robot_ip, const std::string &gripper_ip, std::mutex &mutex_panda);
    Eigen::Vector<double, 7> getRobotJointPosition() const;
    void start();
    void stop();

private:
    Eigen::Matrix<double, 2, 3> generateTrajectory(double time, Eigen::Matrix<double, 2, 3> begin, Eigen::Matrix<double, 2, 3> end, double begin_time, double end_time);

    std::string robot_ip_;
    std::string gripper_ip_;
    franka::Robot robot_;
    franka::Gripper gripper_;

    double time_step_;
    Eigen::Matrix<double, 3, 4> traj_point_;
    double trajectory_time_;
    double time_point_[8];
    Eigen::MatrixXd kp_, kd_, ki_;
    Eigen::VectorXd error_sum_;
    Eigen::Vector<double, 7> q_;

    void controlLoop();

    std::thread control_thread_;
    std::mutex &mutex_panda_;
    bool running_;
};

#endif //
