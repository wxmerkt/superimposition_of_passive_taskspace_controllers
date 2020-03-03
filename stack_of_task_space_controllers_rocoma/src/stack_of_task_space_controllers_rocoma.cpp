#include "stack_of_task_space_controllers_rocoma/stack_of_task_space_controllers_rocoma.hpp"
#include <message_logger/message_logger.hpp>

inline double clamp(double x, double lower, double upper)
{
    return std::max(lower, std::min(upper, x));
}

namespace stack_of_task_space_controllers
{
StackOfTaskSpaceControllersRoCo::StackOfTaskSpaceControllersRoCo() : Base()
{
    this->setName("stack_of_task_space_controllers");

    // Create node handle and subscribe...
    ros::NodeHandle n("~");
    controller_stack_.Initialize(n);
}
StackOfTaskSpaceControllersRoCo::~StackOfTaskSpaceControllersRoCo()
{
}

void StackOfTaskSpaceControllersRoCo::setModes()
{
    for (auto& actuatorCommand : getCommand().getActuatorCommands())
    {
        actuatorCommand.setMode(series_elastic_actuator::SeActuatorCommand::MODE_JOINT_TORQUE);
    }
}

bool StackOfTaskSpaceControllersRoCo::initialize(double dt)
{
    this->setIsCheckingState(false);

    MELO_INFO_STREAM("Initialize controller to set current_positions");

    // Initialise real-time thread-safe buffers
    RobotState& robot_current_state = controller_stack_.get_robot_current_state();
    int i = 0;
    std::vector<double> q(n_joints_);
    for (auto& actuator : actuatorEnums_)
    {
        // Protect out-of-range...
        if (i == n_joints_)
            break;
        q[i] = getState().getActuatorReadings()[actuator].getState().getJointPosition();
        robot_current_state.q(i) = q[i];
        ++i;
    }
    controller_stack_.UpdateTargetPosesInPassiveControllers(q);

    setModes();

    return true;
}

bool StackOfTaskSpaceControllersRoCo::reset(double dt)
{
    return initialize(dt);
}

bool StackOfTaskSpaceControllersRoCo::advance(double dt)
{
    setModes();

    // Eigen::VectorXd q(n_joints_), qdot(n_joints_);
    RobotState& current_robot_state = controller_stack_.get_robot_current_state();
    for (int i = 0; i < n_joints_; ++i)
    {
        auto& actuator = actuatorEnums_[i];
        current_robot_state.q(i) = getState().getActuatorReadings()[actuator].getState().getJointPosition();
        current_robot_state.qdot(i) = getState().getActuatorReadings()[actuator].getState().getJointVelocity();
        current_robot_state.tau(i) = getState().getActuatorReadings()[actuator].getState().getJointTorque();
        current_robot_state.q_for_exotica[joint_names_[i]] = current_robot_state.q(i);
    }
    controller_stack_.UpdateCurrentStateFromRobotState();
    // controller_stack_.UpdateCurrentState(q, qdot);

    Eigen::VectorXd tau_command = controller_stack_.ComputeCommandTorques();
    for (int i = 0; i < n_joints_; ++i)
    {
        auto& actuator = actuatorEnums_[i];
        getCommand().getActuatorCommands()[actuator].setJointTorque(clamp(tau_command(i), -40., 40.));
        ++i;
    }

    return true;
}
} /* namespace stack_of_task_space_controllers */
