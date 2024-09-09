#include "sai2/control.hpp"

Control::Control(shared_ptr<Sai2Model::Sai2Model> robot,
                 shared_ptr<Sai2Simulation::Sai2Simulation> sim,
                 std::mutex &mutex_control,
                 std::mutex &mutex_simulation,
                 double control_freq)
    : _robot(robot), _sim(sim), _mutex_control(mutex_control), _mutex_simulation(mutex_simulation), _control_freq(control_freq), _fControlRunning(true)
{
    Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.07);
    Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));
    string link_name = "end-effector";

    _timer = make_unique<Sai2Common::LoopTimer>(_control_freq);
    _robot_name = "PANDA"; // Adjust this based on your actual robot name

    _fControlRunning = true;
    _control_torques = Vector<double, 7>::Zero();
    _ui_torques = Vector<double, 7>::Zero();
    _robot->updateModel();
    _initial_q = _robot->q();
    _dof = _robot->dof();

    _motion_force_task = make_unique<Sai2Primitives::MotionForceTask>(_robot, link_name, compliant_frame);
    _joint_task = make_unique<Sai2Primitives::JointTask>(_robot);
    _motion_force_task->disableInternalOtg();
    _motion_force_task->enableVelocitySaturation();
    _joint_task->setGains(100, 20);
}

Control::~Control()
{
    stop(); // Ensure that the thread is stopped before destruction
}

void Control::start()
{
    _thread = std::thread(&Control::controlLoop, this);
}

void Control::stop()
{
    _fControlRunning = false;
    if (_thread.joinable())
    {
        _thread.join(); // Wait for the thread to finish execution
    }
}

Vector<double, 7> Control::getTorque() const
{
    lock_guard<mutex> lock(_mutex_control);
    return this->_control_torques;
}

void Control::controlLoop()
{
    MatrixXd N_prec;

    Vector<double, 7> motion_force_task_torques = Vector<double, 7>::Zero();
    Vector<double, 7> joint_task_torques = Vector<double, 7>::Zero();
    vector<Vector3d> desired_offsets{Vector3d(0.2, 0, 0), Vector3d(0, 0, 0),
                                     Vector3d(0, 0.2, 0), Vector3d(0, 0, 0),
                                     Vector3d(0, -0.2, 0), Vector3d(0, 0, 0),
                                     Vector3d(0, 0, 0.2), Vector3d(0, 0, 0)};

    vector<double> t_wait{5, 5};
    double prev_time = 0;
    int cnt = 0;
    int max_cnt = desired_offsets.size();

    while (_fControlRunning)
    {
        _timer->waitForNextLoop();
        const double time = _timer->elapsedSimTime();

        {
            lock_guard<mutex> lock(_mutex_simulation);
            _robot->setQ(_sim->getJointPositions(_robot_name));
            _robot->setDq(_sim->getJointVelocities(_robot_name));
            _robot->updateModel();
        }

        N_prec = MatrixXd::Identity(_dof, _dof);
        _motion_force_task->updateTaskModel(N_prec);
        N_prec = _motion_force_task->getTaskAndPreviousNullspace();
        _joint_task->updateTaskModel(N_prec);
        if (time - prev_time > t_wait[cnt % 2])
        {
            _motion_force_task->setGoalPosition(desired_offsets[cnt]);
            cnt++;
            prev_time = time;
            if (cnt == max_cnt)
                cnt = max_cnt - 1;
        }
        _motion_force_task->setGoalLinearVelocity(Vector3d::Zero());
        _motion_force_task->setGoalLinearAcceleration(Vector3d::Zero());

        motion_force_task_torques = _motion_force_task->computeTorques();
        joint_task_torques = _joint_task->computeTorques();

        {
            lock_guard<mutex> lock(_mutex_control);
            _control_torques = motion_force_task_torques + joint_task_torques;
        }
    }

    _timer->stop();
    cout << "\nControl loop timer stats:\n";
    _timer->printInfoPostRun();
}
