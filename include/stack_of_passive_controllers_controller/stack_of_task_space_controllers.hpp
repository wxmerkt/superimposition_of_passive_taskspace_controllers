#ifndef STACK_OF_PASSIVE_CONTROLLERS_STACK_OF_TASK_SPACE_CONTROLLERS_HPP_
#define STACK_OF_PASSIVE_CONTROLLERS_STACK_OF_TASK_SPACE_CONTROLLERS_HPP_

#include <string>
#include <vector>

#include <Eigen/Dense>

// #include <angles/angles.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ddynamic_reconfigure/param/dd_all_params.h>
#include <realtime_tools/realtime_buffer.h>
// #include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

#include <exotica_core/exotica_core.h>

#include "stack_of_passive_controllers_controller/fractal_impedance_controller.hpp"

namespace stack_of_passive_controllers_controller
{
struct RobotState
{
    RobotState() {}
    RobotState(const std::vector<std::string>& joint_names)
        : q(Eigen::VectorXd::Zero(joint_names.size())), qdot(Eigen::VectorXd::Zero(joint_names.size())), tau(Eigen::VectorXd::Zero(joint_names.size()))
    {
        for (const auto& joint_name : joint_names)
        {
            q_for_exotica[joint_name] = 0.0;
        }
    }

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd tau;
    std::map<std::string, double> q_for_exotica;
};

class StackOfTaskSpaceControllersController
{
public:
    StackOfTaskSpaceControllersController(ros::NodeHandle& n);
    ~StackOfTaskSpaceControllersController();

    // Method to update the internal state of the target task space poses from a joint configuration vector
    void UpdateTargetPosesInPassiveControllers(const std::vector<double>& q);

    // Method to update the current state of the robot (joint configuration vector)
    // void UpdateCurrentState(const std::vector<double>& q, const std::vector<double>& qdot);
    // Requires the state to be updated in robot_current_state_
    void UpdateCurrentStateFromRobotState();

    // Expose the reference to the memory object (don't mess with this!)
    RobotState& get_robot_current_state() { return robot_current_state_; }
    // Method to initialise controller - called before switching (upon start)
    // Start

    // Method to stop controller, i.e., unsubscribe
    // Stop

    // Method to calculate torque to command based on internal information on state and targets
    Eigen::VectorXd ComputeCommandTorques();

protected:
    std::vector<std::string> joint_names_;
    std::size_t n_joints_;
    double joint_damping_ = 0.0;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    // Exotica Scenes for Kinematics - a separate one for updates from the command subscriber and one for the control loop (as data is not yet separate).
    exotica::ScenePtr scene_control_loop_;
    exotica::ScenePtr scene_subscriber_;

    // Dynamic reconfigure
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

    // Stack of controllers
    std::vector<PassiveController> passive_controllers_;

    // Debug publishers
    std::map<std::string, ros::Publisher> pub_fic_;

    // Robot joint state
    RobotState robot_current_state_;

    // Smoothing
    // bool first_joint_state_ = true;
    // Eigen::VectorXd last_tau_;
    // Eigen::VectorXd last_q_;
    // Eigen::VectorXd last_qdot_;
    // Vector6d last_error_;
    // Vector6d last_dX_;

    // double alpha_tau_ = 1.0;
    // double alpha_error_ = 1.0;
    // double alpha_dX_ = 1.0;
    // double alpha_q_ = 1.0;
    // double alpha_qdot_ = 1.0;

    // Subscriber for new commands (real-time safe)
    ros::Subscriber sub_command_;
    void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);

    // Reconfigure callback
    static void ddrCB(const ddynamic_reconfigure::DDMap& map, int, StackOfTaskSpaceControllersController* obj);
};
}  // namespace stack_of_passive_controllers_controller

#endif  // STACK_OF_PASSIVE_CONTROLLERS_STACK_OF_TASK_SPACE_CONTROLLERS_HPP_