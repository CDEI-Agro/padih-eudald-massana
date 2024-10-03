// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_agribot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_agribot
{
  hardware_interface::CallbackReturn DiffDriveAgribotHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.transmission_ratio_wheels = std::stof(info_.hardware_parameters["transmission_ratio_wheels"]);
    cfg_.encoder_ticks_per_rev = std::stoi(info_.hardware_parameters["encoder_ticks_per_rev"]);
    cfg_.can_mode = std::stoi(info_.hardware_parameters["can_mode"]);
    cfg_.can_filter = std::stoi(info_.hardware_parameters["can_filter"]);
    cfg_.can_loop_time = std::stoi(info_.hardware_parameters["can_loop_time"]);


    wheel_l_.setup(cfg_.left_wheel_name);
    wheel_r_.setup(cfg_.right_wheel_name);

    //Configure the CAN parameters (timing, baud rate, etc.)
    memset(&can_config_, 0, sizeof(VCI_INIT_CONFIG));
    can_config_.AccCode = 0x80000008;      // Acceptance code
    can_config_.AccMask = 0xFFFFFFFF;      // Acceptance mask
    can_config_.Filter = cfg_.can_filter;      // Filter mode: 1 = single filter, 0 = dual filter
    can_config_.Mode = cfg_.can_mode;    // Mode: 0 = normal, 1 = listen-only
    can_config_.Timing0 = 0x00;           // CAN baud rate (adjust these values for your CAN bus)
    can_config_.Timing1 = 0x1C;          // Corresponds to 500 kbps


    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveAgribotHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveAgribotHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveAgribotHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveAgribotHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveAgribotHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveAgribotHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveAgribotHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveAgribotHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Opening CAN DEVICE ...please wait...");

    // Have to check why the OpenCanDevice funtion returns 0 in any case but still opens the device
    if(canalystii_.open_can_device(0) == 0){
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Opened CAN DEVICE");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveAgribotHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Cleaning up ...please wait...");


    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveAgribotHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Initializing and Starting CAN device ...please wait...");


    if(canalystii_.init_can_device(can_config_)==1){
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Initialized CAN Device");
    }

    if(canalystii_.start_can_device(0)==1){
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Started CAN Device");
    }

      // Start the CAN send thread
    stop_can_thread_ = false;
    can_send_thread_ = std::thread(&DiffDriveAgribotHardware::can_send_loop, this);
 
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  void DiffDriveAgribotHardware::can_send_loop() {
  while (!stop_can_thread_) {
    {
      // Lock the mutex to safely access shared data
      std::lock_guard<std::mutex> lock(can_mutex_);

      // Send CAN messages for the wheel and turret commands
      canalystii_.send_can_message(4, wheel_l_.cmd, wheel_r_.cmd );

    }

    // Sleep for a small amount of time to avoid busy-waiting (e.g., 10ms)
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.can_loop_time));
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "CAN send thread stopped.");
}

  hardware_interface::CallbackReturn DiffDriveAgribotHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Deactivating ...please wait...");
    
      // Stop the CAN thread
    stop_can_thread_ = true;
    if (can_send_thread_.joinable()) {
    can_send_thread_.join();  // Wait for the thread to finish
    }

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Successfully deactivated CAN thread!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveAgribotHardware::on_shutdown(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

      // Stop the CAN thread
  stop_can_thread_ = true;
  if (can_send_thread_.joinable()) {
    can_send_thread_.join();  // Wait for the thread to finish
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveAgribotHardware"), "Successfully shutdown.");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffDriveAgribotHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

      // updating wheel commands at the controller's loop frequency


  // canalystii_.receive_can_message(4,0,0);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type diffdrive_agribot ::DiffDriveAgribotHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

          // these commands will be read in the can_Send_loop and sent to can bus at the can_send_loop's freqeuncy (cfg._can_loop_time)
      // std::lock_guard<std::mutex> lock(can_mutex_);

      // if we need to do some computations with the wheel commands we will do them here.
      // wheel_l_cmd_ = wheel_l_.cmd;
      // wheel_r_cmd_ = wheel_r_.cmd;
    return hardware_interface::return_type::OK;
  }

} // namespace diffdrive_agribot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diffdrive_agribot::DiffDriveAgribotHardware, hardware_interface::SystemInterface)
