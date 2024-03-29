// Copyright 2020 Wolfgang Merkt

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ddynamic_reconfigure/param/dd_all_params.h>
#include <realtime_tools/realtime_buffer.h>
// #include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/filters.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

#include <ipab_lwr_msgs/FriCommandJointStiffness.h>
#include <ipab_lwr_msgs/FriCommandMode.h>
#include <sensor_msgs/JointState.h>

#include <exotica_core/exotica_core.h>

// #include "1euro_filter.hpp"

inline double clamp(double x, double lower, double upper)
{
    return std::max(lower, std::min(upper, x));
}

inline double sign(double x)
{
    if (x > 0)
        return +1.0;
    else if (x == 0)
        return 0.0;
    else
        return -1.0;
}

struct MovingAverageFilter
{
    MovingAverageFilter() {}
    MovingAverageFilter(std::size_t dimension, std::size_t num_history) : history(Eigen::MatrixXd::Zero(dimension, num_history)), init_(true) {}
    Eigen::VectorXd filter(const Eigen::VectorXd& val)
    {
        if (!init_)
        {
            ROS_ERROR_STREAM("Filter not initialized!!!");
            return val;
        }
        // Shift history left
        for (int i = history.cols() - 1; i > 0; --i)
        {
            history.col(i - 1) = history.col(i);
        }
        // Add new value to right-most column
        history.col(history.cols() - 1) = val;

        // Compute average and return
        return history.rowwise().mean();
    }

    Eigen::MatrixXd history;
    bool init_ = false;
};

namespace stack_of_task_space_controllers
{
typedef Eigen::Matrix<double, 6, 1> Vector6d;

struct PassiveController
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PassiveController(const std::string& link_name_in,
                      const Eigen::Ref<const Vector6d>& Wmax_in,
                      const Eigen::Ref<const Vector6d>& Errb_in,
                      const Eigen::Ref<const Vector6d>& Err0_in,
                      const Eigen::Ref<const Vector6d>& K0_in)
        : link_name(link_name_in),
          controller_name(link_name_in),
          Wmax(Wmax_in),
          Errb(Errb_in),
          Err0(Err0_in),
          K0(K0_in),
          Wmax_scale(1.0),
          Wmax_init_(Wmax_in),
          K0_init_(K0_in)
    {
    }

    // Returns F, xmax
    std::pair<Vector6d, Vector6d> FractalImpedanceControl(const Eigen::Ref<const Vector6d>& Xmax, const Eigen::Ref<const Vector6d>& dX, const Eigen::Ref<const Vector6d>& Err)
    {
        F_.setZero();
        xmax_.setZero();

        // Update scaled variables
        Wmax = Wmax_scale * Wmax_init_;
        K0 = Wmax_scale * K0_init_;

        for (int i = 0; i < 6; ++i)
        {
            constexpr double capacitor_charge = 20.;
            const double b = (Errb(i) - Err0(i)) / capacitor_charge;

            constexpr double cartesian_velocity_tolerance = 1e-2;
            if (sign(Err(i)) == -sign(dX(i)) || std::abs(dX(i)) < cartesian_velocity_tolerance)
            {
                xmax_(i) = Err(i);
                if (std::abs(Err(i)) < Err0(i))
                {
                    F_(i) = K0(i) * Err(i);
                }
                else
                {
                    F_(i) = sign(Err(i)) * ((1. - std::exp(-(std::abs(Err(i)) - Err0(i)) / b)) * (Wmax(i) - K0(i) * Err0(i)) + K0(i) * Err0(i));
                }
            }
            else
            {
                if (std::isfinite(Xmax(i)))
                {
                    xmax_(i) = Xmax(i);
                }
                else
                {
                    xmax_(i) = Err(i);
                }

                double xmid = xmax_(i) / 2.;
                if (std::abs(xmid) < 1e-6)
                {
                    F_(i) = 0.;
                    continue;
                }
                // double AbsXmax = std::abs(xmax_(i));

                double U;
                if (std::abs(xmax_(i)) < Err0(i))
                {
                    U = 0.5 * K0(i) * xmax_(i) * xmax_(i);
                }
                else
                {
                    U = Wmax(i) * std::abs(xmax_(i)) - Wmax(i) * Err0(i) + (K0(i) * Err0(i) * Err0(i)) / 2. -
                        b * (Wmax(i) - K0(i) * Err0(i)) +
                        b * std::exp(-(std::abs(xmax_(i)) - Err0(i)) / b) * (Wmax(i) - K0(i) * Err0(i));
                }

                double Kout = U / (xmid * xmid);
                F_(i) = -Kout * (xmid - Err(i));

                if (std::abs(F_(i)) > Wmax(i))
                {
                    F_(i) = -sign(xmid - Err(i)) * Wmax(i);
                }
            }
        }

        return {F_, xmax_};
    }

    std::string link_name;
    std::string controller_name;
    Vector6d Wmax;
    Vector6d Errb;
    Vector6d Err0;
    Vector6d K0;
    Eigen::Vector3d link_offset = Eigen::Vector3d::Zero();
    KDL::Frame link_offset_frame = KDL::Frame();

    void SetLinkOffset(const Eigen::Ref<const Eigen::Vector3d>& offset)
    {
        link_offset = offset;
        link_offset_frame = KDL::Frame(KDL::Vector(offset(0), offset(1), offset(2)));
    }

    Vector6d tmp_Xmax = Vector6d::Zero();
    double Wmax_scale = 1.0;
    // Original variables
    Vector6d Wmax_init_;
    Vector6d K0_init_;

    realtime_tools::RealtimeBuffer<KDL::Frame> target_pose;
    // Vector6d target_pose;
    Vector6d current_pose;

    bool base_for_orientation_control = false;
    double orientation_scale = 1.0;

private:
    Vector6d F_;
    Vector6d xmax_;
};

class StackOfPassiveControllersController
{
public:
    StackOfPassiveControllersController() {}
    ~StackOfPassiveControllersController()
    {
        Eigen::VectorXd tau_final_ = Eigen::VectorXd::Zero(n_joints_);
        commandTorques(tau_final_);
        switchToPositionControl();
        sub_command_.shutdown();
    }

    bool init(std::shared_ptr<ros::NodeHandle> n)
    {
        n_ = n;

        // List of controlled joints
        if (!n->getParam("joints", joint_names_))
        {
            ROS_ERROR_STREAM("Failed to get list of joints (namespace: " << n->getNamespace() << ").");
            return false;
        }
        n_joints_ = joint_names_.size();

        // Get URDF
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse URDF file");
            return false;
        }

        // Initialise Exotica scene for forward kinematics and Jacobians
        exotica::Server::InitRos(n);
        scene_control_loop_ = exotica::Setup::CreateScene(exotica::SceneInitializer());
        scene_subscriber_ = exotica::Setup::CreateScene(exotica::SceneInitializer());

        // ddynamic_reconfigure
        ros::NodeHandle nh_priv("~");
        ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(nh_priv));

        if (n_joints_ == 0)
        {
            ROS_ERROR_STREAM("List of joint names is empty.");
            return false;
        }
        for (std::size_t i = 0; i < n_joints_; i++)
        {
            const auto& joint_name = joint_names_[i];

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }

        // Offset in command buffer
        XmlRpc::XmlRpcValue passive_controllers;
        if (!n->getParam("passive_controllers", passive_controllers))
        {
            ROS_ERROR_STREAM("No passive controllers defined, exiting.");
            return false;
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
                    ROS_ERROR_STREAM("The passive controller for '" << link_name << "' does not define '" << item << "'");
                    return false;
                }

                std::vector<double> values;
                if (!n->getParam("passive_controllers/" + link_name + "/" + item, values))
                {
                    ROS_ERROR_STREAM("Could not get param '" << item << "' for link '" << link_name << "'.");
                    return false;
                }
                if (values.size() != static_cast<std::size_t>(6))
                {
                    ROS_ERROR_STREAM(item << " has wrong size. Needs to be 6.");
                    return false;
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
            n->param<bool>("passive_controllers/" + link_name + "/Orientation", orientation_control, false);
            new_passive_controller.base_for_orientation_control = orientation_control;
            passive_controllers_.push_back(new_passive_controller);

            // Set up three position controllers if orientation control desired
            if (orientation_control)
            {
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
            pub_fic_[link_name] = n->advertise<std_msgs::Float64MultiArray>("/stack_of_fic/" + link_name + "/force", 1);
        }
        ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_tau", 0, "alpha_tau", 0.95, 0, 1));
        ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_dX", 0, "alpha_dX", 1.0, 0, 1));
        ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_error", 0, "alpha_error", 1.0, 0, 1));
        ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_q", 0, "alpha_q", 0.95, 0, 1));
        ddr_->add(new ddynamic_reconfigure::DDDouble("alpha_qdot", 0, "alpha_qdot", 0.95, 0, 1));

        // Initialise real-time thread-safe buffers
        std::vector<double> q(n_joints_);
        q.assign(n_joints_, 0.0);
        q[3] = 1.57;
        UpdateTargetPosesInPassiveControllers(q);

        // Subscribe to command topic
        sub_command_ = n->subscribe<std_msgs::Float64MultiArray>("command", 1, &StackOfPassiveControllersController::commandCB, this);
        sub_joint_state_ = n->subscribe<sensor_msgs::JointState>("/joint_states", 1, &StackOfPassiveControllersController::jointStateCB, this);
        pub_joint_command_mode_ = n->advertise<ipab_lwr_msgs::FriCommandMode>("/lwr/commandMode", 1);
        pub_joint_stiffness_command_ = n->advertise<ipab_lwr_msgs::FriCommandJointStiffness>("/lwr/commandJointStiffness", 1);

        // Init tau for smoothing
        last_tau_ = Eigen::VectorXd::Zero(n_joints_);
        last_q_ = Eigen::VectorXd::Zero(n_joints_);
        last_qdot_ = Eigen::VectorXd::Zero(n_joints_);
        last_error_ = Vector6d::Zero();
        last_dX_ = Vector6d::Zero();
        filter_ = MovingAverageFilter(n_joints_, 5);

        // static const double duration = 10;
        // static const double frequency = 1/.003;
        // static const double mincutoff = 1;
        // static const double beta = 1;
        // static const double dcutoff = 1;
        // filter_.reset(new one_euro_filter<>(frequency, mincutoff, beta, dcutoff));

        // Publish ddynamic_reconfigure

        // v0.2.0 (pal-robotics)
        // ddr_->publishServicesTopics();

        // awesomebytes (& ANYbotics) / RealSense
        // ddr_->start(&StackOfPassiveControllersController::ddrCB, this);
        // if (!first_start_done_)
        // {
        //     first_start_done_ = true;
        //     ddr_->start(boost::bind(ddrCB, _1, _2, this));
        // }

        return true;
    }

protected:
    std::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher pub_joint_command_mode_;
    ros::Publisher pub_joint_stiffness_command_;
    std::map<std::string, ros::Publisher> pub_fic_;

    bool first_start_done_ = false;
    std::vector<std::string> joint_names_;
    std::size_t n_joints_;

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    // Exotica Scenes for Kinematics - a separate one for updates from the command subscriber and one for the control loop (as data is not yet separate).
    exotica::ScenePtr scene_control_loop_;
    exotica::ScenePtr scene_subscriber_;

    // Dynamic reconfigure
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;

    // Stack of controllers
    std::vector<PassiveController> passive_controllers_;

    // Subscriber for new commands (real-time safe)
    ros::Subscriber sub_command_;
    void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
        // Positions
        if (msg->data.size() == n_joints_)
        {
            UpdateTargetPosesInPassiveControllers(msg->data);
        }
        else
        {
            ROS_ERROR_STREAM("Number of desired positions wrong: got " << msg->data.size() << " expected " << n_joints_ << " or 0.");
        }
    }

    ros::Subscriber sub_joint_state_;
    sensor_msgs::JointState joint_state_;
    bool first_joint_state_ = true;
    Eigen::VectorXd last_tau_;
    Eigen::VectorXd last_q_;
    Eigen::VectorXd last_qdot_;
    Vector6d last_error_;
    Vector6d last_dX_;
    // std::unique_ptr<one_euro_filter<>> filter_;
    MovingAverageFilter filter_;
    double alpha_tau_ = 1.0;
    double alpha_error_ = 1.0;
    double alpha_dX_ = 1.0;
    double alpha_q_ = 1.0;
    double alpha_qdot_ = 1.0;
    void jointStateCB(const sensor_msgs::JointStateConstPtr& msg)
    {
        if (msg->position.size() != n_joints_)
        {
            ROS_ERROR("Sizes don't match!!!!!!");
            return;
        }

        joint_state_ = *msg;

        // If this is the first joint state, update the target pose to current... to avoid crazy behaviour.
        if (first_joint_state_)
        {
            first_joint_state_ = false;
            UpdateTargetPosesInPassiveControllers(msg->position);

            switchToTorqueControl();
        }

        if (!first_start_done_)
        {
            first_start_done_ = true;
            ddr_->start(boost::bind(ddrCB, _1, _2, this));
        }

        // TODO: preallocate all the things.

        // Build message to update Exotica
        Eigen::VectorXd q(n_joints_);     // = Eigen::Map<Eigen::VectorXd>(msg->position.data(), n_joints_);
        Eigen::VectorXd qdot(n_joints_);  // = Eigen::Map<Eigen::VectorXd>(msg->velocity.data(), n_joints_);
        // Eigen::VectorXd tau_measured = Eigen::Map<Eigen::VectorXd>(msg->effort.data(), n_joints_);
        std::map<std::string, double> q_for_exotica;
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            q(i) = msg->position[i];
            qdot(i) = msg->velocity[i];

            // Filter q, qdot
            q(i) = filters::exponentialSmoothing(q(i), last_q_(i), alpha_q_);
            qdot(i) = filters::exponentialSmoothing(qdot(i), last_qdot_(i), alpha_qdot_);

            q_for_exotica[joint_names_[i]] = q(i);  // TODO: Use msg->names
        }
        last_q_ = q;
        last_qdot_ = qdot;
        scene_control_loop_->GetKinematicTree().SetModelState(q_for_exotica);

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
            Vector6d dX = current_link_jacobian * qdot;

            // Filter error
            for (int i = 0; i < 6; ++i)
            {
                error(i) = filters::exponentialSmoothing(error(i), last_error_(i), alpha_error_);
                dX(i) = filters::exponentialSmoothing(dX(i), last_dX_(i), alpha_dX_);
            }
            last_error_ = error;
            last_dX_ = dX;
            ROS_WARN_STREAM_THROTTLE(1, passive_controller.link_name << ": " << error.transpose());

            auto FIC = passive_controller.FractalImpedanceControl(passive_controller.tmp_Xmax, dX, error);
            passive_controller.tmp_Xmax = FIC.second;

            // Map back to joint space
            tau += current_link_jacobian.transpose() * FIC.first;
            std_msgs::Float64MultiArray msg;
            msg.data.resize(6);
            for (int i = 0; i < 6; ++i) msg.data[i] = FIC.first(i);
            pub_fic_[passive_controller.link_name].publish(msg);
            // ROS_ERROR_STREAM(passive_controller.link_name << ": error=" << error.transpose() << ", dX=" << dX.transpose() << ", FIC=" << FIC.first.transpose());
        }

        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            tau(i) = clamp(tau(i), -joint_urdfs_[i]->limits->effort, joint_urdfs_[i]->limits->effort);
            // Smooth
            tau(i) = filters::exponentialSmoothing(tau(i), last_tau_(i), alpha_tau_);
            // last_tau_(i) = tau(i);
            // tau(i) = filter_->filter(static_cast<double>(tau(i)), msg->header.stamp.toSec());
        }
        // Smooth
        // tau = filter_.filter(tau);

        ROS_ERROR_STREAM_THROTTLE(1, "tau: " << tau.transpose());
        commandTorques(tau);
    }

    void commandTorques(const Eigen::VectorXd& tau)
    {
        ipab_lwr_msgs::FriCommandJointStiffness msg;
        msg.header.stamp = ros::Time::now();
        msg.jointDamping.assign(n_joints_, 0.);
        msg.jointStiffness.assign(n_joints_, 0.);
        msg.jointPosition.resize(n_joints_);
        msg.jointTorque.resize(n_joints_);
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            msg.jointPosition[i] = static_cast<float>(joint_state_.position[i]);
            // TODO: Cancel gravity compensation (!)
            msg.jointTorque[i] = static_cast<float>(tau(i));
        }
        pub_joint_stiffness_command_.publish(msg);
        // string[] joint_names - unset as controller does not strictly require them
    }

    void switchControlMode(int mode)
    {
        ipab_lwr_msgs::FriCommandMode msg;
        msg.header.stamp = ros::Time::now();
        msg.controlMode = mode;
        pub_joint_command_mode_.publish(msg);
    }

    void switchToPositionControl()
    {
        switchControlMode(10);
    }

    void switchToTorqueControl()
    {
        switchControlMode(30);
    }

    // ddynamic_reconfigure callback
    static void ddrCB(const ddynamic_reconfigure::DDMap& map, int, StackOfPassiveControllersController* obj)
    {
        obj->alpha_tau_ = ddynamic_reconfigure::get(map, std::string("alpha_tau").c_str()).toDouble();
        obj->alpha_dX_ = ddynamic_reconfigure::get(map, std::string("alpha_dX").c_str()).toDouble();
        obj->alpha_error_ = ddynamic_reconfigure::get(map, std::string("alpha_error").c_str()).toDouble();
        obj->alpha_q_ = ddynamic_reconfigure::get(map, std::string("alpha_q").c_str()).toDouble();
        obj->alpha_qdot_ = ddynamic_reconfigure::get(map, std::string("alpha_qdot").c_str()).toDouble();
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

    // Method to update all targets from a joint configuration
    // void UpdateTargetPosesInPassiveControllers(const std::map<std::string, double>& q)
    void UpdateTargetPosesInPassiveControllers(const std::vector<double>& q)
    {
        std::map<std::string, double> q_exotica;
        for (std::size_t i = 0; i < n_joints_; ++i)
            q_exotica[joint_names_[i]] = q[i];
        scene_subscriber_->GetKinematicTree().SetModelState(q_exotica);
        for (auto& passive_controller : passive_controllers_)
        {
            KDL::Frame tmp = scene_subscriber_->GetKinematicTree().FK(passive_controller.link_name, passive_controller.link_offset_frame, "", KDL::Frame());
            // ROS_INFO_STREAM("Updating " << passive_controller.link_name << " to " << tmp.transpose());
            passive_controller.target_pose.writeFromNonRT(tmp);
            // passive_controller.target_pose = tmp;
        }
        // ROS_INFO_STREAM_THROTTLE(1, "Target poses updated");
    }
};
}  // namespace stack_of_task_space_controllers

int main(int argc, char** argv)
{
    ROS_INFO_STREAM("Starting Stack of Fractal Impedance Controllers");

    ros::init(argc, argv, "stack_of_fic_node");
    std::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle());

    stack_of_task_space_controllers::StackOfPassiveControllersController fic;
    if (!fic.init(n))
    {
        ROS_ERROR_STREAM("Could not initialize :'(");
        exit(-1);
    }

    ros::AsyncSpinner spinner(4);  // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
}
