/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by peter on 2021/7/17.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <serial.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "protocol.h"

#include <rm2_msgs/msg/shoot_cmd.hpp>
#include <rm2_msgs/msg/shoot_state.hpp>
#include <rm2_msgs/msg/dbus_data.hpp>
#include <rm2_msgs/msg/state_cmd.hpp>
#include <rm2_msgs/msg/event_data.hpp>
#include <rm2_msgs/msg/gimbal_cmd.hpp>
#include <rm2_msgs/msg/robot_hurt.hpp>
#include <rm2_msgs/msg/shoot_data.hpp>
#include <rm2_msgs/msg/dart_status.hpp>
#include <rm2_msgs/msg/chassis_cmd.hpp>
#include <rm2_msgs/msg/game_status.hpp>
#include <rm2_msgs/msg/rfid_status.hpp>
#include <rm2_msgs/msg/engineer_ui.hpp>
#include <rm2_msgs/msg/game_robot_hp.hpp>
#include <rm2_msgs/msg/balance_state.hpp>
#include <rm2_msgs/msg/dart_client_cmd.hpp>
#include <rm2_msgs/msg/actuator_state.hpp>
#include <rm2_msgs/msg/map_sentry_data.hpp>
#include <rm2_msgs/msg/radar_mark_data.hpp>
#include <rm2_msgs/msg/power_heat_data.hpp>
#include <rm2_msgs/msg/gimbal_des_error.hpp>
#include <rm2_msgs/msg/bullet_allowance.hpp>
#include <rm2_msgs/msg/game_robot_status.hpp>
#include <rm2_msgs/msg/manual_to_referee.hpp>
#include <rm2_msgs/msg/client_map_send_data.hpp>
#include <rm2_msgs/msg/robots_position_data.hpp>
#include <rm2_msgs/msg/dart_remaining_time.hpp>
#include <rm2_msgs/srv/status_change.hpp>
#include <rm2_msgs/msg/client_map_receive_data.hpp>
#include <rm2_msgs/msg/supply_projectile_action.hpp>
#include <rm2_msgs/msg/icra_buff_debuff_zone_status.hpp>
#include <rm2_msgs/msg/game_robot_pos_data.hpp>
#include "rm2_msgs/msg/sentry_info.hpp"
#include "rm2_msgs/msg/radar_info.hpp"
#include "rm2_msgs/msg/buff.hpp"
#include "rm2_msgs/msg/track_data.hpp"
#include "rm2_msgs/msg/sentry_attacking_target.hpp"
#include <rm2_msgs/msg/power_management_sample_and_status_data.hpp>
#include <rm2_msgs/msg/power_management_system_exception_data.hpp>
#include <rm2_msgs/msg/power_management_initialization_exception_data.hpp>
#include <rm2_msgs/msg/power_management_process_stack_overflow_data.hpp>
#include <rm2_msgs/msg/power_management_unknown_exception_data.hpp>

namespace rm2_referee
{
struct CapacityData
{
  double chassis_power;
  double limit_power;
  double buffer_power;
  double cap_power;
  bool is_online = false;
};

class Base
{
public:
  serial::Serial serial_;
  int client_id_ = 0;  // recipient's id
  int robot_id_ = 0;   // recent  robot's id
  int capacity_recent_mode_, capacity_expect_mode_;
  std::string robot_color_;
  bool referee_data_is_online_ = false;

  void initSerial()
  {
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setPort("/dev/usbReferee");
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
    if (serial_.isOpen())
      return;
    try
    {
      serial_.open();
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR("Cannot open referee port");
    }
  }

  // CRC check
  uint8_t getCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length, unsigned char uc_crc_8)
  {
    unsigned char uc_index;
    while (dw_length--)
    {
      uc_index = uc_crc_8 ^ (*pch_message++);
      uc_crc_8 = rm2_referee::kCrc8Table[uc_index];
    }
    return (uc_crc_8);
  }

  uint32_t verifyCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length)
  {
    unsigned char uc_expected;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return 0;
    uc_expected = getCRC8CheckSum(pch_message, dw_length - 1, rm2_referee::kCrc8Init);
    return (uc_expected == pch_message[dw_length - 1]);
  }

  void appendCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length)
  {
    unsigned char uc_crc;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return;
    uc_crc = getCRC8CheckSum((unsigned char*)pch_message, dw_length - 1, rm2_referee::kCrc8Init);
    pch_message[dw_length - 1] = uc_crc;
  }

  uint16_t getCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length, uint16_t w_crc)
  {
    uint8_t chData;
    if (pch_message == nullptr)
      return 0xFFFF;
    while (dw_length--)
    {
      chData = *pch_message++;
      (w_crc) = (static_cast<uint16_t>(w_crc) >> 8) ^
                rm2_referee::wCRC_table[(static_cast<uint16_t>(w_crc) ^ static_cast<uint16_t>(chData)) & 0x00ff];
    }
    return w_crc;
  }

  uint32_t verifyCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length)
  {
    uint16_t w_expected;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return 0;
    w_expected = getCRC16CheckSum(pch_message, dw_length - 2, rm2_referee::kCrc16Init);
    return ((w_expected & 0xff) == pch_message[dw_length - 2] &&
            ((w_expected >> 8) & 0xff) == pch_message[dw_length - 1]);
  }

  void appendCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length)
  {
    uint16_t wCRC;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return;
    wCRC = getCRC16CheckSum(static_cast<uint8_t*>(pch_message), dw_length - 2, rm2_referee::kCrc16Init);
    pch_message[dw_length - 2] = static_cast<uint8_t>((wCRC & 0x00ff));
    pch_message[dw_length - 1] = static_cast<uint8_t>(((wCRC >> 8) & 0x00ff));
  }
};
}  // namespace rm2_referee
