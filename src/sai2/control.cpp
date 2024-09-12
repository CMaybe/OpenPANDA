#include "sai2/control.hpp"

Control::Control(shared_ptr<Sai2Model::Sai2Model> robot,
                 shared_ptr<Sai2Simulation::Sai2Simulation> sim,
                 std::mutex &mutex_control,
                 std::mutex &mutex_simulation,
                 double control_freq)
    : robot_(robot), sim_(sim), mutex_control_(mutex_control), mutex_simulation_(mutex_simulation), control_freq_(control_freq), fControlRunning_(true)
{
    Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.07);
    Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));
    string link_name = "end-effector";

    timer_ = make_unique<Sai2Common::LoopTimer>(control_freq_);
    robot_name_ = "PANDA";  // Adjust this based on your actual robot name

    fControlRunning_ = true;
    control_torques_ = Vector<double, 7>::Zero();
    ui_torques_ = Vector<double, 7>::Zero();
    robot_->updateModel();
    initial_q_ = robot_->q();
    dof_ = robot_->dof();

    motion_force_task_ = make_unique<Sai2Primitives::MotionForceTask>(robot_, link_name, compliant_frame);
    joint_task_ = make_unique<Sai2Primitives::JointTask>(robot_);
    motion_force_task_->disableInternalOtg();
    motion_force_task_->enableVelocitySaturation();
    joint_task_->setGains(100, 20);
}

Control::~Control()
{
    stop();  // Ensure that the thread is stopped before destruction
}

void Control::start()
{
    thread_ = std::thread(&Control::controlLoop, this);
}

void Control::stop()
{
    fControlRunning_ = false;
    if (thread_.joinable())
    {
        thread_.join();  // Wait for the thread to finish execution
    }
}

Vector<double, 7> Control::getTorque() const
{
    lock_guard<mutex> lock(mutex_control_);
    return this->control_torques_;
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

    while (fControlRunning_)
    {
        timer_->waitForNextLoop();
        const double time = timer_->elapsedSimTime();

        {
            lock_guard<mutex> lock(mutex_simulation_);
            robot_->setQ(sim_->getJointPositions(robot_name_));
            robot_->setDq(sim_->getJointVelocities(robot_name_));
            robot_->updateModel();
        }

        N_prec = MatrixXd::Identity(dof_, dof_);
        motion_force_task_->updateTaskModel(N_prec);
        N_prec = motion_force_task_->getTaskAndPreviousNullspace();
        joint_task_->updateTaskModel(N_prec);
        if (time - prev_time > t_wait[cnt % 2])
        {
            motion_force_task_->setGoalPosition(desired_offsets[cnt]);
            cnt++;
            prev_time = time;
            if (cnt == max_cnt)
                cnt = max_cnt - 1;
        }
        motion_force_task_->setGoalLinearVelocity(Vector3d::Zero());
        motion_force_task_->setGoalLinearAcceleration(Vector3d::Zero());

        motion_force_task_torques = motion_force_task_->computeTorques();
        joint_task_torques = joint_task_->computeTorques();

        {
            lock_guard<mutex> lock(mutex_control_);
            control_torques_ = motion_force_task_torques + joint_task_torques;
        }
    }

    timer_->stop();
    cout << "\nControl loop timer stats:\n";
    timer_->printInfoPostRun();
}
