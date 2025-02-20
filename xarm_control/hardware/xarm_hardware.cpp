// Copyright 2023 ros2_control Development Team
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

#include "xarm_control/xarm_hardware.hpp"
#include <string>
#include <vector>
#include <math.h>

#include <iostream>

namespace xarm_control
{
  CallbackReturn XArmSystemHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // config_.reduction = stoi(info_.hardware_parameters["reduction"]);
    std::string joints_zero = info_.hardware_parameters["zero_pos"];

    int idx;

    for (int i = 0; i < 6; i++)
    {
      idx = joints_zero.find(" ");
      config_.zero_pos[i] = stod(joints_zero.substr(0, idx));
      joints_zero = joints_zero.substr(idx + 1);
    }

    joint_1_.setup(config_.reduction);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn XArmSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // reset values always when configuring hardware
    for (const auto &[name, descr] : joint_state_interfaces_)
    {
      set_state(name, 0.0);
    }
    for (const auto &[name, descr] : joint_command_interfaces_)
    {
      set_command(name, 0.0);
    }
    for (const auto &[name, descr] : sensor_state_interfaces_)
    {
      set_state(name, 0.0);
    }

    RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Configuring ...please wait...");

    comms_.connect(config_.device);

    RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Successfully configured!");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn XArmSystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Cleaning up ...please wait...");

    comms_.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Successfully cleaned up!");

    return CallbackReturn::SUCCESS;
  }

  return_type XArmSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    double enc_pose[6] = {0.0};
    bool read_state = false;

    read_state = comms_.read_encoder_values(enc_pose[0], enc_pose[1], enc_pose[2], enc_pose[3],
                                            enc_pose[4], enc_pose[5]);

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (info_.joints.size() == 6)
      {
        if (i < 5)
        {
          enc_pose[i] = (enc_pose[i] - config_.zero_pos[i]) * M_PI / 18000.0; // Convert to radians
        }
        else if (i == 5)
        {
          // 16248 - 4848
          enc_pose[i] = (-(enc_pose[i] - config_.zero_pos[i]) * M_PI) / (18000.0 * 1.328); // Convert to radians
        }

        if (read_state)
        {
          const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
          const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;

          const auto mirr_vel = info_.joints[5].name + "/" + hardware_interface::HW_IF_VELOCITY;
          const auto mirr_pos = info_.joints[5].name + "/" + hardware_interface::HW_IF_POSITION;

          if (i < 6)
          {
            set_state(name_vel, (enc_pose[i] - get_state(name_pos)) / period.seconds());
            set_state(name_pos, enc_pose[i]);
          }
          else if (i == 6 or i == 7)
          {
            set_state(name_vel, get_state(mirr_vel) * -1);
            set_state(name_pos, get_state(mirr_pos) * -1);
          }
          else if (i == 8)
          {
            set_state(name_vel, get_state(mirr_vel) * 1);
            set_state(name_pos, get_state(mirr_pos) * 1);
          }
        }
        else
        {
          const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
          const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;

          set_state(name_vel, get_state(name_vel));
          set_state(name_pos, get_state(name_pos));
        }
      }
      else
        std::cout << "Not enough joints: " << info_.joints.size() << std::endl;
    }

    return return_type::OK;
  }

  return_type XArmSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &period)
  {
    int cmd[7];

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;

      double joint_command = get_command(name_pos);

      if (i < 5)
      {
        joint_command = (joint_command * 18000 / M_PI) + config_.zero_pos[i]; // Convert to radians
      }
      else
      {
        joint_command = -(joint_command * 18000 * 1.328 / M_PI) + config_.zero_pos[i]; // Convert to radians
      }

      cmd[i] = int(joint_command);
    }

    cmd[info_.joints.size()] = int(period.seconds() * 1000.0);

    comms_.set_values(cmd);
    return return_type::OK;
  }

} // namespace xarm_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    xarm_control::XArmSystemHardware, hardware_interface::SystemInterface)
