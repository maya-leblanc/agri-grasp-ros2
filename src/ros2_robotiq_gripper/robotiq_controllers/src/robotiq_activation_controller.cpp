// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "pluginlib/class_list_macros.hpp" 
#include "robotiq_controllers/robotiq_activation_controller.hpp"

#include <chrono>
#include <thread>

namespace robotiq_controllers
{
controller_interface::InterfaceConfiguration RobotiqActivationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("reactivate_gripper/reactivate_gripper_cmd");
  config.names.emplace_back("reactivate_gripper/reactivate_gripper_response");

  return config;
}

controller_interface::InterfaceConfiguration RobotiqActivationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::return_type RobotiqActivationController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RobotiqActivationController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotiqActivationController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != 2)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 2 command interfaces, but got %zu.", command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  try
  {
    reactivate_gripper_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/reactivate_gripper",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
          this->reactivateGripper(req, resp);
        });
  }
  catch (...)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotiqActivationController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    reactivate_gripper_srv_.reset();
  }
  catch (...)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool RobotiqActivationController::reactivateGripper(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  (void)req;

  // EDITED: Evaluate the nodiscard bool return value of set_value to clear compiler warnings
  bool success = command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].set_value(ASYNC_WAITING);
  success &= command_interfaces_[REACTIVATE_GRIPPER_CMD].set_value(1.0);
  
  resp->success = success;

  while (true)
  {
    // EDITED: Migrated from deprecated get_value() to get_optional() for ROS 2 Jazzy/Harmonic standard compatibility
    const auto current_opt = command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].get_optional();
    const double current_value = current_opt.value_or(0.0);

    if (current_value != ASYNC_WAITING)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // EDITED: Handled final assignment using safe optional retrieval fallback
  const auto final_opt = command_interfaces_[REACTIVATE_GRIPPER_RESPONSE].get_optional();
  resp->success &= static_cast<bool>(final_opt.value_or(0.0));

  return resp->success;
}
}  // namespace robotiq_controllers

PLUGINLIB_EXPORT_CLASS(robotiq_controllers::RobotiqActivationController, controller_interface::ControllerInterface)