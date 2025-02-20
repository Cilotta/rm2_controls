//
// Created by yezi on 2022/3/26.
//

#pragma once

#include "rm2_common/filters/imu_filter_base.h"
#include <imu_complementary_filter/complementary_filter_ros.h>

namespace rm2_common
{
class ImuComplementaryFilter : public ImuFilterBase
{
public:

  ImuComplementaryFilter() = default;
  void getOrientation(double& q0, double& q1, double& q2, double& q3) override;

private:

  void filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt) override;
  bool initFilter(rclcpp::Parameter& imu_data) override;
  void resetFilter() override;
  // Parameters:
  double gain_acc_;
  double gain_mag_;
  bool do_bias_estimation_;
  double bias_alpha_;
  bool do_adaptive_gain_;
  bool use_mag_;
  std::shared_ptr<imu_tools::ComplementaryFilter> filter_;
};
}  // namespace rm2_common
