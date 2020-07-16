#ifndef STACK_OF_TASK_SPACE_CONTROLLERS_ROCOMA_STACK_OF_TASK_SPACE_CONTROLLERS_ROCOMA_HPP_
#define STACK_OF_TASK_SPACE_CONTROLLERS_ROCOMA_STACK_OF_TASK_SPACE_CONTROLLERS_ROCOMA_HPP_

#include <anymal_roco/anymal_roco.hpp>
#include <roco/controllers/controllers.hpp>

#include <anymal_description/AnymalDescription.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <stack_of_task_space_controllers_core/stack_of_task_space_controllers.hpp>

namespace stack_of_task_space_controllers
{
class StackOfTaskSpaceControllersRoCo : virtual public roco::Controller<anymal_roco::RocoState, anymal_roco::RocoCommand>
{
public:
    using AD = anymal_description::AnymalDescription;
    using Base = roco::Controller<anymal_roco::RocoState, anymal_roco::RocoCommand>;

    StackOfTaskSpaceControllersRoCo();
    virtual ~StackOfTaskSpaceControllersRoCo();

    bool create(double dt) override { return true; }
    bool initialize(double dt) override;
    bool reset(double dt) override;
    bool advance(double dt) override;
    bool cleanup() override { return true; }
    bool stop() override { return true; }
    bool preStop() override { return true; }
protected:
    void setModes();
    StackOfTaskSpaceControllers controller_stack_;

    std::vector<AD::LimbEnum> limbEnums_ = {AD::LimbEnum::LF_LEG, AD::LimbEnum::RF_LEG, AD::LimbEnum::LH_LEG, AD::LimbEnum::RH_LEG};  // ETHZ
    std::vector<AD::ActuatorNodeEnum> actuatorNodeEnums_ = {AD::ActuatorNodeEnum::HAA, AD::ActuatorNodeEnum::HFE, AD::ActuatorNodeEnum::KFE};
    std::vector<AD::ActuatorEnum> actuatorEnums_ = {
        AD::ActuatorEnum::LF_HAA, AD::ActuatorEnum::LF_HFE, AD::ActuatorEnum::LF_KFE,
        AD::ActuatorEnum::LH_HAA, AD::ActuatorEnum::LH_HFE, AD::ActuatorEnum::LH_KFE,
        AD::ActuatorEnum::RF_HAA, AD::ActuatorEnum::RF_HFE, AD::ActuatorEnum::RF_KFE,
        AD::ActuatorEnum::RH_HAA, AD::ActuatorEnum::RH_HFE, AD::ActuatorEnum::RH_KFE};
    std::vector<std::string> joint_names_ = {
        "LF_HAA", "LF_HFE", "LF_KFE",
        "RF_HAA", "RF_HFE", "RF_KFE",
        "LH_HAA", "LH_HFE", "LH_KFE",
        "RH_HAA", "RH_HFE", "RH_KFE",
    };
    int n_joints_ = 12;

    // Default values
    std::vector<double> default_zero_vector_ = std::vector<double>(n_joints_, 0.0);

    // Real-time buffers
    realtime_tools::RealtimeBuffer<std::vector<double>> desired_efforts_buffer_;
};

} /* namespace stack_of_task_space_controllers */

#endif /* STACK_OF_TASK_SPACE_CONTROLLERS_ROCOMA_STACK_OF_TASK_SPACE_CONTROLLERS_ROCOMA_HPP_ */
