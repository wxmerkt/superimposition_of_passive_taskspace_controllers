#include "rocoma_plugin/rocoma_plugin.hpp"
#include "stack_of_task_space_controllers_rocoma/stack_of_task_space_controllers_rocoma.hpp"

ROCOMA_EXPORT_CONTROLLER(StackOfTaskSpaceControllersRoCo, anymal_roco::RocoState,
                         anymal_roco::RocoCommand,
                         stack_of_task_space_controllers::StackOfTaskSpaceControllersRoCo)
