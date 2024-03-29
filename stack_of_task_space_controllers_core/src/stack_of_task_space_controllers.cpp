#include "stack_of_task_space_controllers_core/stack_of_task_space_controllers.hpp"

namespace stack_of_task_space_controllers
{
inline double clamp(double x, double lower, double upper)
{
    return std::max(lower, std::min(upper, x));
}

void StackOfTaskSpaceControllers::Initialize(ros::NodeHandle& n)
{
    ROS_INFO_STREAM("Init!");

    // List of controlled joints
    if (!n.getParam("joints", joint_names_))
    {
        ThrowPretty("Failed to get list of joints (namespace: " << n.getNamespace() << ").");
    }
    n_joints_ = joint_names_.size();
    robot_current_state_ = RobotState(joint_names_);

    // Joint damping (e.g. for simulation)
    if (!n.getParam("joint_damping", joint_damping_))
    {
        joint_damping_ = 0.;
    }

    // Get URDF
    urdf::Model urdf;
    if (!urdf.initParam("robot_description"))
    {
        ThrowPretty("Failed to parse URDF file");
    }

    // Initialise Exotica scene for forward kinematics and Jacobians
    exotica::Server::InitRos(std::shared_ptr<ros::NodeHandle>(&n));
    scene_control_loop_ = exotica::Setup::CreateScene(exotica::SceneInitializer());
    scene_subscriber_ = exotica::Setup::CreateScene(exotica::SceneInitializer());

    // ddynamic_reconfigure
    ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(n));

    if (n_joints_ == 0)
    {
        ThrowPretty("List of joint names is empty.");
    }
    for (std::size_t i = 0; i < n_joints_; i++)
    {
        const auto& joint_name = joint_names_[i];

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        }
        joint_urdfs_.push_back(joint_urdf);
    }

    // Offset in command buffer
    XmlRpc::XmlRpcValue passive_controllers;
    if (!n.getParam("passive_controllers", passive_controllers))
    {
        ThrowPretty("No passive controllers defined, exiting.");
    }
    for (auto& passive_controller : passive_controllers)
    {
        ROS_INFO_STREAM("Passive controller defined for '" << passive_controller.first << "'");
        const std::string& link_name = passive_controller.first;

        std::map<std::string, Vector6d> parsed_values;
        for (auto& item : {"Wmax", "Err0", "Errb", "K0"})
        {
            if (!passive_controller.second.hasMember(item))
            {
                ThrowPretty("The passive controller for '" << link_name << "' does not define '" << item << "'");
            }

            std::vector<double> values;
            if (!n.getParam("passive_controllers/" + link_name + "/" + item, values))
            {
                ThrowPretty("Could not get param '" << item << "' for link '" << link_name << "'.");
            }
            if (values.size() != static_cast<std::size_t>(6))
            {
                ThrowPretty(item << " has wrong size. Needs to be 6.");
            }
            parsed_values.insert(std::make_pair<std::string, Vector6d>(item, Eigen::Map<Vector6d>(values.data(), values.size())));
        }
        PassiveController new_passive_controller(link_name,
                                                 parsed_values["Wmax"],
                                                 parsed_values["Errb"],
                                                 parsed_values["Err0"],
                                                 parsed_values["K0"]);

        // Check if orientation control is desired
        bool orientation_control = false;
        n.param<bool>("passive_controllers/" + link_name + "/Orientation", orientation_control, false);
        new_passive_controller.base_for_orientation_control = orientation_control;
        passive_controllers_.push_back(new_passive_controller);

        // Set up three position controllers if orientation control desired
        if (orientation_control)
        {
            ThrowPretty("No!");
            PassiveController ctrl_1(link_name + "_rot_1", parsed_values["Wmax"], parsed_values["Errb"], parsed_values["Err0"], parsed_values["K0"]);
            ctrl_1.SetLinkOffset(Eigen::Vector3d(1, 0, 0));
            ctrl_1.link_name = link_name;
            passive_controllers_.push_back(ctrl_1);

            PassiveController ctrl_2(link_name + "_rot_2", parsed_values["Wmax"], parsed_values["Errb"], parsed_values["Err0"], parsed_values["K0"]);
            ctrl_2.SetLinkOffset(Eigen::Vector3d(0, 1, 0));
            ctrl_2.link_name = link_name;
            passive_controllers_.push_back(ctrl_2);

            PassiveController ctrl_3(link_name + "_rot_3", parsed_values["Wmax"], parsed_values["Errb"], parsed_values["Err0"], parsed_values["K0"]);
            ctrl_3.SetLinkOffset(Eigen::Vector3d(0, 0, 1));
            ctrl_3.link_name = link_name;
            passive_controllers_.push_back(ctrl_3);
        }

        // Add scale parameter to dynamic reconfigure
        // v0.2.0 (pal-robotics)
        // ddr_->registerVariable<double>(link_name + "/Wmax_scale", &new_passive_controller.Wmax_scale, link_name + "/Wmax_scale (for Wmax and K0)");

        // awesomebytes (& ANYbotics)
        if (new_passive_controller.base_for_orientation_control) ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/OrientationScale", 0, link_name + "/OrientationScale (relative to position)", 0.2, 0, 5));
        ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/Wmax_scale", 0, link_name + "/Wmax_scale (for Wmax and K0)", 1, 0, 100));
        ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/Wmax_trans", 0, link_name + "/Wmax_trans", parsed_values["Wmax"](0), 0, 1000));
        ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/K0_trans", 0, link_name + "/K0_trans", parsed_values["K0"](0), 0, 10000));
        ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/Err0_trans", 0, link_name + "/Err0_trans", parsed_values["Err0"](0), 0, 0.2));
        ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/Errb_trans", 0, link_name + "/Errb_trans", parsed_values["Errb"](0), 0, 0.2));
        if (!orientation_control)
        {
            ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/Wmax_rot", 0, link_name + "/Wmax_rot", parsed_values["Wmax"](3), 0, 100));
            ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/K0_rot", 0, link_name + "/K0_rot", parsed_values["K0"](3), 0, 10000));
            ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/Err0_rot", 0, link_name + "/Err0_rot", parsed_values["Err0"](3), 0, 0.2));
            ddr_->add(new ddynamic_reconfigure::DDDouble(link_name + "/Errb_rot", 0, link_name + "/Errb_rot", parsed_values["Errb"](3), 0, 0.2));
        }

        // Add debug publishers
        pub_fic_[link_name] = n.advertise<std_msgs::Float64MultiArray>(link_name + "/computed_force", 1);
    }
    ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_tau", 0, "alpha_tau", 0.95, 0, 1));
    ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_dX", 0, "alpha_dX", 1.0, 0, 1));
    ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_error", 0, "alpha_error", 1.0, 0, 1));
    ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_q", 0, "alpha_q", 0.95, 0, 1));
    ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_qdot", 0, "alpha_qdot", 0.95, 0, 1));

    initialized_ = true;

    // Initialise real-time thread-safe buffers
    std::vector<double> q(n_joints_);
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
        q[i] = 0.0;
    }
    UpdateTargetPosesInPassiveControllers(q);

    // Subscribe to command topic
    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &StackOfTaskSpaceControllers::commandCB, this);

    // Subscribe to Cartesian command targets
    sub_cartesian_commands_.reserve(passive_controllers_.size());
    for (const auto& passive_controller : passive_controllers_)
    {
        // sub_cartesian_commands_.emplace_back(n.subscribe<geometry_msgs::Point>(passive_controller.link_name + "/cartesian_target", 1, &StackOfTaskSpaceControllers::cartesian_commandCB, this));
        const std::string topic_name = passive_controller.link_name + "/cartesian_target";
        sub_cartesian_commands_.emplace_back(n.subscribe<geometry_msgs::Point>(topic_name, 1, boost::bind(&StackOfTaskSpaceControllers::cartesian_commandCB, this, _1, topic_name)));
    }

    // Joint limits
    // joint_limits_interface::JointLimits limits;

    ddr_->start(boost::bind(ddrCB, _1, _2, this));
}

// Method to update all targets from a joint configuration
void StackOfTaskSpaceControllers::UpdateTargetPosesInPassiveControllers(const std::vector<double>& q)
{
    if (!initialized_) ThrowPretty("Not initialized.");

    std::map<std::string, double> q_exotica;
    for (std::size_t i = 0; i < n_joints_; ++i)
        q_exotica[joint_names_[i]] = q[i];
    scene_subscriber_->GetKinematicTree().SetModelState(q_exotica);
    // auto q_tmp = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    // HIGHLIGHT_NAMED("UpdateTargetPosesInPassiveControllers", q_tmp.transpose())
    for (auto& passive_controller : passive_controllers_)
    {
        KDL::Frame tmp = scene_subscriber_->GetKinematicTree().FK(passive_controller.link_name, passive_controller.link_offset_frame, "", KDL::Frame());
        // ROS_INFO_STREAM("Updating " << passive_controller.link_name << " to " << tmp.transpose());
        passive_controller.target_pose.writeFromNonRT(tmp);
        // passive_controller.target_pose = tmp;
    }
}

void StackOfTaskSpaceControllers::UpdateCurrentStateFromRobotState()  //(const std::vector<double>& q, const std::vector<double>& qdot)
{
    if (!initialized_) ThrowPretty("Not initialized.");

    // if (q.size() != joint_names_.size() || qdot.size() != joint_names_.size())
    // {
    //     ThrowPretty("Size mismatch: " << q.size() << " vs " << joint_names_.size());
    // }

    // for (std::size_t i = 0; i < n_joints_; ++i)
    // {
    //     robot_current_state_.q(i) = q[i];
    //     robot_current_state_.qdot(i) = qdot[i];
    //     // robot_current_state_.tau(i) = joints_[i].getEffort();
    //     robot_current_state_.q_for_exotica.begin() + .at(i).second = q(i);
    // }
    scene_control_loop_->GetKinematicTree().SetModelState(robot_current_state_.q_for_exotica);
}

Eigen::VectorXd StackOfTaskSpaceControllers::ComputeCommandTorques()
{
    if (!initialized_) ThrowPretty("Not initialized.");

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(n_joints_);
    for (auto& passive_controller : passive_controllers_)
    {
        // Skip orientation controllers if they are deactivated.
        if (passive_controller.orientation_scale == 0.0) continue;

        // Get current link positions
        KDL::Frame current_link_position = scene_control_loop_->GetKinematicTree().FK(passive_controller.link_name, passive_controller.link_offset_frame, "", KDL::Frame());
        Eigen::MatrixXd current_link_jacobian = scene_control_loop_->GetKinematicTree().Jacobian(passive_controller.link_name, passive_controller.link_offset_frame, "", KDL::Frame()).block(0, 0, 6, n_joints_);  // NASTY SUBSET SELECTION!!!!

        // Compute error
        KDL::Frame target_pose = *passive_controller.target_pose.readFromRT();
        KDL::Twist error_twist = KDL::diff(current_link_position, target_pose);
        Vector6d error;
        error.head<3>() = Eigen::Map<Eigen::Vector3d>(error_twist.vel.data);
        error.tail<3>() = Eigen::Map<Eigen::Vector3d>(error_twist.rot.data);
        Vector6d dX = current_link_jacobian * robot_current_state_.qdot;

        // Filter error
        // for (int i = 0; i < 6; ++i)
        // {
        //     error(i) = filters::exponentialSmoothing(error(i), last_error_(i), alpha_error_);
        //     dX(i) = filters::exponentialSmoothing(dX(i), last_dX_(i), alpha_dX_);
        // }
        // last_error_ = error;
        // last_dX_ = dX;
        // ROS_WARN_STREAM_THROTTLE(1, passive_controller.link_name << ": " << error.transpose());

        auto FIC = passive_controller.FractalImpedanceControl(passive_controller.tmp_Xmax, dX, error);
        passive_controller.tmp_Xmax = FIC.second;

        // Map back to joint space

        tau.noalias() += current_link_jacobian.transpose() * FIC.first;
        std_msgs::Float64MultiArray msg;
        msg.data.resize(6);
        for (int i = 0; i < 6; ++i) msg.data[i] = FIC.first(i);
        pub_fic_[passive_controller.link_name].publish(msg);
        if (control_tick_ % 500 == 0)
        {
            const Eigen::VectorXd tau_this_level = current_link_jacobian.transpose() * FIC.first;
            HIGHLIGHT_NAMED(passive_controller.link_name, std::setw(8) << std::fixed << std::setprecision(4)
                                                                       << " Goal:" << Eigen::Map<Eigen::Vector3d>(target_pose.p.data).transpose()
                                                                       << "\tError=" << error.head<3>().transpose()
                                                                       << "\tFIC=" << FIC.first.head<3>().transpose()
                                                                       << "\ttau=" << tau_this_level.transpose());
        }
        // ROS_ERROR_STREAM(passive_controller.link_name << ": error=" << error.transpose() << ", dX=" << dX.transpose() << ", FIC=" << FIC.first.transpose());
    }

    // Add joint damping
    tau -= joint_damping_ * robot_current_state_.qdot;

    Eigen::VectorXd tau_command = tau;
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
        tau_command(i) = clamp(tau(i), -joint_urdfs_[i]->limits->effort, joint_urdfs_[i]->limits->effort);
    }

    control_tick_++;

    return tau_command;
}

// ddynamic_reconfigure callback
void StackOfTaskSpaceControllers::ddrCB(const ddynamic_reconfigure::DDMap& map, int, StackOfTaskSpaceControllers* obj)
{
    for (auto& passive_controller : obj->passive_controllers_)
    {
        passive_controller.Wmax_scale = ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Wmax_scale").c_str()).toDouble();

        passive_controller.Wmax_init_.head<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Wmax_trans").c_str()).toDouble());
        passive_controller.K0_init_.head<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/K0_trans").c_str()).toDouble());

        passive_controller.Err0.head<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Err0_trans").c_str()).toDouble());
        passive_controller.Errb.head<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Errb_trans").c_str()).toDouble());

        if (passive_controller.base_for_orientation_control)
        {
            passive_controller.orientation_scale = ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/OrientationScale").c_str()).toDouble();

            for (auto suffix : {"_rot_1", "_rot_2", "_rot_3"})
            {
                for (auto& pc : obj->passive_controllers_)
                {
                    if (pc.controller_name == passive_controller.link_name + suffix)
                    {
                        pc.Wmax_scale = passive_controller.orientation_scale * ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Wmax_scale").c_str()).toDouble();
                        pc.Wmax_init_.head<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Wmax_trans").c_str()).toDouble());
                        // pc.Wmax_init_.tail<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Wmax_rot").c_str()).toDouble());
                        pc.K0_init_.head<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/K0_trans").c_str()).toDouble());
                        // pc.K0_init_.tail<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/K0_rot").c_str()).toDouble());

                        pc.Err0.head<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Err0_trans").c_str()).toDouble());
                        pc.Errb.head<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Errb_trans").c_str()).toDouble());
                    }
                }
            }
        }
        else
        {
            passive_controller.Wmax_init_.tail<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Wmax_rot").c_str()).toDouble());
            passive_controller.K0_init_.tail<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/K0_rot").c_str()).toDouble());
            passive_controller.Err0.tail<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Err0_rot").c_str()).toDouble());
            passive_controller.Errb.tail<3>().setConstant(ddynamic_reconfigure::get(map, std::string(passive_controller.link_name + "/Errb_rot").c_str()).toDouble());
        }
    }
}

void StackOfTaskSpaceControllers::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    // Positions
    if (msg->data.size() == n_joints_)
    {
        // ROS_DEBUG_STREAM_THROTTLE_NAMED(1., "Pose Optimization Update", "New joint angles received.");
        UpdateTargetPosesInPassiveControllers(msg->data);
    }
    else
    {
        ROS_ERROR_STREAM("Number of desired positions wrong: got " << msg->data.size() << " expected " << n_joints_ << " or 0.");
    }
}

// Cartesian command
void StackOfTaskSpaceControllers::cartesian_commandCB(const geometry_msgs::PointConstPtr& msg, const std::string& topic)
{
    std::string controller_name(topic);
    const std::string string_to_erase = "/cartesian_target";
    controller_name.erase(topic.length() - string_to_erase.length(), string_to_erase.length());

    // ROS_INFO_STREAM("Received new cartesian command on topic = " << topic << ", controller = " << controller_name);
    for (auto& passive_controller : passive_controllers_)
    {
        if (passive_controller.link_name == controller_name)
        {
            // KDL::Frame target_pose = *passive_controller.target_pose.readFromRT();
            // ROS_INFO_STREAM("Updating " << controller_name << " to x=" << msg->x << ", y=" << msg->y << ", z=" << msg->z << " (from: " << Eigen::Map<Eigen::Vector3d>(target_pose.p.data).transpose() << ")");
            KDL::Frame tmp(KDL::Vector(msg->x, msg->y, msg->z));
            passive_controller.target_pose.writeFromNonRT(tmp);
            return;
        }
    }
    ROS_ERROR_STREAM("No corresponding controller found for " << controller_name);
}
}  // namespace stack_of_task_space_controllers
