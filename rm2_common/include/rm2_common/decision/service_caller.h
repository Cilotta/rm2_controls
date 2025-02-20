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
// Created by qiayuan on 5/22/21.
//

#pragma once

#include <chrono>
#include <mutex>
#include <thread>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <control_msgs/srv/query_calibration_state.hpp>
#include <rm2_msgs/srv/status_change.hpp>

namespace rm2_common
{
template <class ServiceType>
class ServiceCallerBase
{
public:
  explicit ServiceCallerBase(rclcpp::Node& nh, const std::string& service_name = "") : fail_count_(0), fail_limit_(0)
  {
    nh.param("fail_limit", fail_limit_, 0);
    if (!nh.param("service_name", service_name_, service_name) && service_name.empty())
    {
      ROS_ERROR("Service name no defined (namespace: %s)", nh.getNamespace().c_str());
      return;
    }
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }

  ServiceCallerBase(rclcpp::Node& nh, std::string& service_name) : fail_count_(0), fail_limit_(0)
  {
    service_name_ = service_name;
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }

  ServiceCallerBase(rclcpp::Parameter& controllers, rclcpp::Node& nh, const std::string& service_name = "") : fail_count_(0), fail_limit_(0)
  {
    if (controllers->has_Member("service_name"))
      service_name_ = static_cast<std::string>(controllers["service_name"]);
    else 
    {
      service_name_ = service_name;
      if (service_name.empty())
      {
        ROS_ERROR("Service name no defined (namespace: %s)", nh.getNamespace().c_str());
        return;
      }
    }
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }

  ~ServiceCallerBase()
  {
    delete thread_;
  }

  void callService()
  {
    if (isCalling())
      return;
    thread_ = new std::thread(&ServiceCallerBase::callingThread, this);
    thread_->detach();
  }

  ServiceType& getService()
  {
    return service_;
  }

  bool isCalling()
  {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    return !guard.owns_lock();
  }

protected:

  void callingThread()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!client_.call(service_))
    {
      ROS_INFO_ONCE("Failed to call service %s on %s. Retrying now ...", typeid(ServiceType).name(),
                    service_name_.c_str());
      if (fail_limit_ != 0)
      {
        fail_count_++;
        if (fail_count_ >= fail_limit_)
        {
          ROS_ERROR_ONCE("Failed to call service %s on %s", typeid(ServiceType).name(), service_name_.c_str());
          fail_count_ = 0;
        }
      }
      //      ros::WallDuration(0.2).sleep();
    }
  }
  
  std::string service_name_;
  ros::ServiceClient client_;
  ServiceType service_request_;
  std::thread* thread_{};
  std::mutex mutex_;
  int fail_count_, fail_limit_;
};

class SwitchControllersServiceCaller : public ServiceCallerBase<controller_manager_msgs::srv::SwitchController_Request>
{
public:
  
  explicit SwitchControllersServiceCaller(rclcpp::Node& nh) : ServiceCallerBase<controller_manager_msgs::srv::SwitchController_Request>(nh, "/controller_manager/switch_controller")
  {
    service_request_.strictness = service_request_.BEST_EFFORT;
    service_request_.start_asap = true;
  }
  void startControllers(const std::vector<std::string>& controllers)
  {
    service_request_.start_controllers = controllers;
  }

  void stopControllers(const std::vector<std::string>& controllers)
  {
    service_request_.stop_controllers = controllers;
  }

  bool getOk()
  {
    if (isCalling())
      return false;
    return service_request_.ok;
  }
};

class QueryCalibrationServiceCaller : public ServiceCallerBase<control_msgs::srv::QueryCalibrationState>
{
public:
  explicit QueryCalibrationServiceCaller(rclcpp::Node& nh)
    : ServiceCallerBase<control_msgs::srv::QueryCalibrationState>(nh)
  {
  }

  QueryCalibrationServiceCaller(rclcpp::Node& nh, std::string& service_name)
    : ServiceCallerBase<control_msgs::srv::QueryCalibrationState>(nh, service_name)
  {
  }
  
  QueryCalibrationServiceCaller(rclcpp::Parameter& controllers, rclcpp::Node& nh)
    : ServiceCallerBase<control_msgs::srv::QueryCalibrationState>(controllers, nh)
  {
  }
  
  bool isCalibrated()
  {
    if (isCalling())
      return false;
    return service_.response.is_calibrated;
  }
};

class SwitchDetectionCaller : public ServiceCallerBase<rm2_msgs::srv::StatusChange>
{
public:
  explicit SwitchDetectionCaller(rclcpp::Node& nh)
    : ServiceCallerBase<rm2_msgs::srv::StatusChange>(nh, "/detection_nodelet/status_switch")
  {
    service_.request.target = rm2_msgs::srv::StatusChange_Request::ARMOR;
    service_.request.exposure = rm2_msgs::srv::StatusChange_Request::EXPOSURE_LEVEL_0;
    service_.request.armor_target = rm2_msgs::srv::StatusChange_Request::ARMOR_ALL;
    callService();
  }

  explicit SwitchDetectionCaller(rclcpp::Node& nh, std::string service_name)
    : ServiceCallerBase<rm2_msgs::srv::StatusChange>(nh, service_name)
  {
    service_.request.target = rm2_msgs::srv::StatusChange_Request::ARMOR;
    service_.request.exposure = rm2_msgs::srv::StatusChange_Request::EXPOSURE_LEVEL_0;
    service_.request.armor_target = rm2_msgs::srv::StatusChange_Request::ARMOR_ALL;
    callService();
  }

  void setEnemyColor(const int& robot_id_, const std::string& robot_color_)
  {
    if (robot_id_ != 0)
    {
      service_.request.color =
          robot_color_ == "blue" ? rm2_msgs::srv::StatusChange_Request::RED : rm2_msgs::srv::StatusChange_Request::BLUE;
      ROS_INFO_STREAM("Set enemy color: " << (service_.request.color == service_.request.RED ? "red" : "blue"));

      callService();
    }
    else
      ROS_INFO_STREAM("Set enemy color failed: referee offline");
  }
  
  void setColor(uint8_t color)
  {
    service_.request.color = color;
  }
  
  void switchEnemyColor()
  {
    service_.request.color = service_.request.color == rm2_msgs::srv::StatusChange_Request::RED;
  }
  
  void switchTargetType()
  {
    service_.request.target = service_.request.target == rm2_msgs::srv::StatusChange_Request::ARMOR;
  }
  
  void setTargetType(uint8_t target)
  {
    service_.request.target = target;
  }
  
  void switchArmorTargetType()
  {
    service_.request.armor_target = service_.request.armor_target == rm2_msgs::srv::StatusChange_Request::ARMOR_ALL;
  }
  
  void setArmorTargetType(uint8_t armor_target)
  {
    service_.request.armor_target = armor_target;
  }
  
  void switchExposureLevel()
  {
    service_.request.exposure = service_.request.exposure == rm2_msgs::srv::StatusChange_Request::EXPOSURE_LEVEL_4 ?
                                    rm2_msgs::srv::StatusChange_Request::EXPOSURE_LEVEL_0 :
                                    service_.request.exposure + 1;
  }
  
  int getColor()
  {
    return service_.request.color;
  }
  
  int getTarget()
  {
    return service_.request.target;
  }
  
  int getArmorTarget()
  {
    return service_.request.armor_target;
  }
  
  uint8_t getExposureLevel()
  {
    return service_.request.exposure;
  }
  
  bool getIsSwitch()
  {
    if (isCalling())
      return false;
    return service_.response.switch_is_success;
  }

private:
  bool is_set_{};
};

}  // namespace rm2_common



