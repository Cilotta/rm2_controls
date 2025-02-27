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
// Created by qiayuan on 12/30/20.
//

#pragma once

#include "rm2_hw/hardware_interface/hardware_interface.h"

#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

namespace rm2_hw
{
using namespace std::chrono;
using clock = high_resolution_clock;

class RmRobotHWLoop
{
public:
  /** \brief Create controller manager. Load loop frequency. Start control loop which call @ref
   * rm2_hw::RmRobotHWLoop::update() in a frequency.
   *
   * @param nh Node-handle of a ROS node.
   * @param hardware_interface A pointer which point to hardware_interface.
   */
  RmRobotHWLoop(ros::NodeHandle& nh, std::shared_ptr<RmRobotHW> hardware_interface);

  ~RmRobotHWLoop();
  /** \brief Timed method that reads current hardware's state, runs the controller code once and sends the new commands
   * to the hardware.
   *
   * Timed method that reads current hardware's state, runs the controller code once and sends the new commands to the
   * hardware.
   *
   * Note: we do not use the TimerEvent time difference because it does NOT guarantee that the time source is strictly
   * linearly increasing.
   */

  void update();

private:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Settings
  double cycle_time_error_threshold_{};

  // Timing
  std::thread loop_thread_;
  std::atomic_bool loop_running_;
  double loop_hz_{};
  ros::Duration elapsed_time_;
  clock::time_point last_time_;

  /** ROS Controller Manager and Runner

      This class advertises a ROS interface for loading, unloading, starting, and
      stopping ros_control-based controllers. It also serializes execution of all
      running controllers in \ref update.
  **/
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Abstract Hardware Interface for your robot
  std::shared_ptr<RmRobotHW> hardware_interface_;
};
}  // namespace rm2_hw
