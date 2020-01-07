// Copyright 2019 Wolfgang Merkt

#include <string>
#include <vector>

#include <Eigen/Dense>

// #include <angles/angles.h>
#include <controller_interface/controller.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ddynamic_reconfigure/param/dd_all_params.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
// #include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

#include <exotica_core/exotica_core.h>

// #include <joint_limits_interface/joint_limits.h>
// #include <joint_limits_interface/joint_limits_urdf.h>
// #include <joint_limits_interface/joint_limits_rosparam.h>

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

namespace stack_of_passive_controllers_controller
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

            if (sign(Err(i)) == -sign(dX(i)) || dX(i) == 0.)
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
                    U = Wmax(i) * std::abs(xmax_(i)) - Wmax(i) * Err0(i) + (K0(i) * Err0(i) * Err0(i)) / 2. -
                        b * (Wmax(i) - K0(i) * Err0(i)) +
                        b * std::exp(-(std::abs(xmax_(i)) - Err0(i)) / b) * (Wmax(i) - K0(i) * Err0(i));

                double Kout = U / (xmid * xmid);
                F_(i) = -Kout * (xmid - Err(i));

                if (std::abs(F_(i)) > Wmax(i))
                    F_(i) = -sign(xmid - Err(i)) * Wmax(i);
            }
        }

        return {F_, xmax_};
    }

    std::string link_name;
    Vector6d Wmax;
    Vector6d Errb;
    Vector6d Err0;
    Vector6d K0;

    Vector6d tmp_Xmax = Vector6d::Zero();
    double Wmax_scale = 1.0;

    realtime_tools::RealtimeBuffer<Vector6d> target_pose;
    // Vector6d target_pose;
    Vector6d current_pose;

private:
    Vector6d F_;
    Vector6d xmax_;

    // Original variables
    Vector6d Wmax_init_;
    Vector6d K0_init_;
};

class StackOfPassiveControllersController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    StackOfPassiveControllersController() {}
    ~StackOfPassiveControllersController() { sub_command_.shutdown(); }
    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
    {
        // List of controlled joints
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR_STREAM("Failed to get list of joints (namespace: " << n.getNamespace() << ").");
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
        exotica::Server::InitRos(std::shared_ptr<ros::NodeHandle>(&n));
        scene_control_loop_ = exotica::Setup::CreateScene(exotica::SceneInitializer());
        scene_subscriber_ = exotica::Setup::CreateScene(exotica::SceneInitializer());

        // ddynamic_reconfigure
        ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(n));

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
        if (!n.getParam("passive_controllers", passive_controllers))
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

                std::vector<double> values;  // = static_cast<std::vector<double>>(passive_controller.second[item]);
                if (!n.getParam("passive_controllers/" + link_name + "/" + item, values))
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
            passive_controllers_.push_back(new_passive_controller);

            // Add scale parameter to dynamic reconfigure
            // v0.2.0 (pal-robotics)
            // ddr_->registerVariable<double>(link_name + "/Wmax_scale", &new_passive_controller.Wmax_scale, link_name + "/Wmax_scale (for Wmax and K0)");

            // awesomebytes (& ANYbotics)
            ddr_->add(new ddynamic_reconfigure::DDDouble(link_name, 0, link_name + "/Wmax_scale (for Wmax and K0)", 1, 0, 100));
        }

        // Initialise real-time thread-safe buffers
        std::vector<double> q(n_joints_);
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            q[i] = joints_[i].getPosition();
        }
        UpdateTargetPosesInPassiveControllers(q);

        // Subscribe to command topic
        sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &StackOfPassiveControllersController::commandCB, this);

        // Joint limits
        // joint_limits_interface::JointLimits limits;

        return true;
    }

    void starting(const ros::Time& time)
    {
        std::vector<double> q_current(n_joints_);
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            q_current[i] = joints_[i].getPosition();
        }
        q_current[3] = 1.57;
        UpdateTargetPosesInPassiveControllers(q_current);
        ROS_INFO_STREAM("Controller starting.");

        // Publish ddynamic_reconfigure

        // v0.2.0 (pal-robotics)
        // ddr_->publishServicesTopics();

        // awesomebytes (& ANYbotics) / RealSense
        // ddr_->start(&StackOfPassiveControllersController::ddrCB, this);
        ddr_->start(boost::bind(ddrCB, _1, _2, this));
    }

    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
    {
        // TODO:: allocate the Xmax for each link...
        Eigen::VectorXd q(n_joints_), qdot(n_joints_);
        std::map<std::string, double> q_for_exotica;
        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            q(i) = joints_[i].getPosition();
            qdot(i) = joints_[i].getVelocity();
            q_for_exotica[joints_[i].getName()] = q(i);
        }
        scene_control_loop_->GetKinematicTree().SetModelState(q_for_exotica);

        Eigen::VectorXd tau = Eigen::VectorXd::Zero(n_joints_);
        for (auto& passive_controller : passive_controllers_)
        {
            // Get current link positions
            Vector6d current_link_position = exotica::GetFrameAsVector(scene_control_loop_->GetKinematicTree().FK(passive_controller.link_name, KDL::Frame(), "", KDL::Frame()), exotica::RotationType::RPY);
            Eigen::MatrixXd current_link_jacobian = scene_control_loop_->GetKinematicTree().Jacobian(passive_controller.link_name, KDL::Frame(), "", KDL::Frame()).block(0, 0, 6, n_joints_);  // NASTY SUBSET SELECTION!!!!

            // Compute error
            Vector6d error = *passive_controller.target_pose.readFromRT() - current_link_position;
            // Vector6d error = passive_controller.target_pose - current_link_position;
            // ROS_ERROR_STREAM("Jac = " << current_link_jacobian.rows() << "x" << current_link_jacobian.cols());
            // ROS_ERROR_STREAM("qdot = " << qdot.rows() << "x" << qdot.cols());
            Vector6d dX = current_link_jacobian * qdot;

            auto FIC = passive_controller.FractalImpedanceControl(passive_controller.tmp_Xmax, dX, error);
            passive_controller.tmp_Xmax = FIC.second;

            // Map back to joint space
            tau += current_link_jacobian.transpose() * FIC.first;
            // ROS_ERROR_STREAM(passive_controller.link_name << ": error=" << error.transpose() << ", dX=" << dX.transpose() << ", FIC=" << FIC.first.transpose());
        }

        // ROS_WARN_STREAM_THROTTLE(1, "Desired torques: " << tau.transpose());
        // ROS_WARN_STREAM("Desired torques: " << tau.transpose());

        for (std::size_t i = 0; i < n_joints_; ++i)
        {
            double effort = tau(i);
            double clamped_effort = clamp(effort, -joint_urdfs_[i]->limits->effort, joint_urdfs_[i]->limits->effort);

            joints_[i].setCommand(clamped_effort);
        }
    }

protected:
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
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

    // ddynamic_reconfigure callback
    static void ddrCB(const ddynamic_reconfigure::DDMap& map, int, StackOfPassiveControllersController* obj)
    {
        HIGHLIGHT("Reconfigure request...")
        for (auto& passive_controller : obj->passive_controllers_)
        {
            passive_controller.Wmax_scale = ddynamic_reconfigure::get(map, std::string(passive_controller.link_name).c_str()).toDouble();
            ROS_WARN_STREAM("Updated " << passive_controller.link_name << " scale to " << passive_controller.Wmax_scale);
        }
    }

    // Method to update all targets from a joint configuration
    void UpdateTargetPosesInPassiveControllers(const std::vector<double>& q)
    {
        std::map<std::string, double> q_exotica;
        for (std::size_t i = 0; i < n_joints_; ++i)
            q_exotica[joint_names_[i]] = q[i];
        scene_subscriber_->GetKinematicTree().SetModelState(q_exotica);
        // auto q_tmp = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
        // HIGHLIGHT_NAMED("UpdateTargetPosesInPassiveControllers", q_tmp.transpose())
        for (auto& passive_controller : passive_controllers_)
        {
            Vector6d tmp = exotica::GetFrameAsVector(scene_subscriber_->GetKinematicTree().FK(passive_controller.link_name, KDL::Frame(), "", KDL::Frame()), exotica::RotationType::RPY);
            // ROS_INFO_STREAM("Updating " << passive_controller.link_name << " to " << tmp.transpose());
            passive_controller.target_pose.writeFromNonRT(tmp);
            // passive_controller.target_pose = tmp;
        }
    }
};
}  // namespace stack_of_passive_controllers_controller

PLUGINLIB_EXPORT_CLASS(stack_of_passive_controllers_controller::StackOfPassiveControllersController, controller_interface::ControllerBase)
