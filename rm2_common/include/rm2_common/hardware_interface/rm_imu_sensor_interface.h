//
// Created by yezi on 2022/10/4.
//
#pragma once

#include <hardware_interface/sensor_interface.hpp>

namespace rm_control
{
class RmImuSensorHandle : public hardware_interface::SensorInterface
{
public:
  RmImuSensorHandle() = default;

  RmImuSensorHandle(const hardware_interface::SensorInterface& imu_sensor_handle, rclcpp::Time* time_stamp)
    : ImuSensorHandle(imu_sensor_handle), time_stamp_(time_stamp)
  {
    if (!time_stamp_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + imu_sensor_handle.getName() +
                                                           "'. Time stamp pointer is null");
    }
  }
  rclcpp::Time getTimeStamp()
  {
    assert(time_stamp_);
    return *time_stamp_;
  }

private:
  rclcpp::Time* time_stamp_ = { nullptr };
};

class RmImuSensorInterface
  : public hardware_interface::HardwareResourceManager<RmImuSensorHandle, hardware_interface::DontClaimResources>
{
};
}  // namespace rm_control
