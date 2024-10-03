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

#ifndef DIFFDRIVE_AGRIBOT__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_AGRIBOT__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diffdrive_agribot/visibility_control.h"

#include "canalystii.hpp"
#include "wheel.hpp"

#include <thread>
#include <mutex>
#include <atomic>

namespace diffdrive_agribot
{
  class DiffDriveAgribotHardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      std::string left_wheel_name = "";
      std::string right_wheel_name = "";
      float transmission_ratio_wheels = 0;
      int encoder_ticks_per_rev = 0;
      int can_mode = 0;
      int can_filter = 0;
      int can_loop_time = 0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveAgribotHardware)

    DIFFDRIVE_AGRIBOT_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    DIFFDRIVE_AGRIBOT_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    canalystii::CANalystii canalystii_;
    VCI_INIT_CONFIG can_config_;
    Config cfg_;
    Wheel wheel_l_;
    Wheel wheel_r_;
    int m_initial_encoder_ticks_l;
    int m_initial_encoder_ticks_r;
    int m_initial_encoder_ticks_turret;

    std::thread can_send_thread_;   // Thread for sending CAN messages
    std::mutex can_mutex_;          // Mutex to protect shared CAN data
    std::atomic<bool> stop_can_thread_;  // Flag to stop the CAN thread

    void can_send_loop();
  };

} // namespace diffdrive_agribot

#endif // DIFFDRIVE_AGRIBOT__DIFFBOT_SYSTEM_HPP_
