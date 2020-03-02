// Copyright 2019-2020 Wolfgang Merkt

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <stack_of_task_space_controllers_core/stack_of_task_space_controllers.hpp>

namespace stack_of_task_space_controllers
{
class StackOfTaskSpaceControllersController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    StackOfTaskSpaceControllersController() {}
    ~StackOfTaskSpaceControllersController() {}
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
    {
        controller_stack_.Initialize(n);

        // List of controlled joints
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR_STREAM("Failed to get list of joints (namespace: " << n.getNamespace() << ").");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }
        for (std::size_t i = 0; i < n_joints_; i++)
        {
            const auto& joint_name = joint_names_[i];

            try
            {
                joints_.push_back(hw->getHandle(joint_name));
            }
            catch (const hardware_interface::HardwareInterfaceException& e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }
        }

        // Initialise real-time thread-safe buffers
        RobotState& robot_current_state = controller_stack_.get_robot_current_state();
        std::vector<double> q(n_joints_);
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            q[i] = joints_[i].getPosition();
            robot_current_state.q(i) = q[i];
        }
        controller_stack_.UpdateTargetPosesInPassiveControllers(q);

        return true;
    }

    void starting(const ros::Time& time)
    {
        std::vector<double> q_current(n_joints_);
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            q_current[i] = joints_[i].getPosition();

            joints_[i].setCommand(joints_[i].getEffort());
        }
        controller_stack_.UpdateTargetPosesInPassiveControllers(q_current);
        ROS_INFO_STREAM("Controller starting.");
    }

    void stopping(const ros::Time& time)
    {
        ROS_WARN_STREAM("Stopping controller.");
    }

    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
    {
        RobotState& current_robot_state = controller_stack_.get_robot_current_state();
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            current_robot_state.q(i) = joints_[i].getPosition();
            current_robot_state.qdot(i) = joints_[i].getVelocity();
            current_robot_state.tau(i) = joints_[i].getEffort();
            current_robot_state.q_for_exotica[joints_[i].getName()] = current_robot_state.q(i);
        }
        last_q_ = current_robot_state.q;
        last_qdot_ = current_robot_state.qdot;
        controller_stack_.UpdateCurrentStateFromRobotState();

        Eigen::VectorXd tau_command = controller_stack_.ComputeCommandTorques();
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            joints_[i].setCommand(tau_command(i));
        }
    }

protected:
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::size_t n_joints_;

    StackOfTaskSpaceControllers controller_stack_;

    // Smoothing
    Eigen::VectorXd last_q_;
    Eigen::VectorXd last_qdot_;
};
}  // namespace stack_of_task_space_controllers

PLUGINLIB_EXPORT_CLASS(stack_of_task_space_controllers::StackOfTaskSpaceControllersController, controller_interface::ControllerBase)
