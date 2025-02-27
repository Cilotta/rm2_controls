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
// Created by qiayuan on 1/5/21.
//

#include "rm2_common/filters/lp_filter.h"
#include "rm2_common/ros_utilities.h" 

LowPassFilter::LowPassFilter(rclcpp::Node& nh) : delta_t_(rclcpp::Duration::from_seconds(0))
{
  nh.get_parameter_or("lp_cutoff_frequency", cutoff_frequency_, -1.0);
  nh.get_parameter_or("lp_debug", is_debug_, false);

  if (is_debug_)
    realtime_pub_ = std::make_shared<realtime_tools::RealtimePublisher<rm2_msgs::msg::LpData>>(nh, "lp_filter", 100);
}

LowPassFilter::LowPassFilter(double cutoff_freq) : delta_t_(rclcpp::Duration::from_seconds(0))
{
  is_debug_ = false;
  cutoff_frequency_ = cutoff_freq;
}

void LowPassFilter::input(double in, rclcpp::Time time)
{
  // My filter reference was Julius O. Smith III, Intro. to Digital Filters
  // With Audio Applications.
  // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
  in_[2] = in_[1];
  in_[1] = in_[0];
  in_[0] = in;

  if (prev_time_.nanoseconds() != 0)  // Not first time through the program
  {
    delta_t_ = time - prev_time_;
    prev_time_ = time;
    if (0 == delta_t_.seconds())
    {
      RCLCPP_ERROR(rclcpp::get_logger("LowPassFilter"), 
                   "delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", 
                   time.seconds());
      return;
    }
  }
  else
  {
    prev_time_ = time;
    return;
  }

  if (cutoff_frequency_ != -1 && cutoff_frequency_ > 0)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt_ = tan((cutoff_frequency_ * 6.2832) * delta_t_.seconds() / 2.);
    // Avoid tan(0) ==> NaN
    if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
      tan_filt_ = -0.01;
    if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
      tan_filt_ = 0.01;
    c_ = 1 / tan_filt_;
  }

  out_[2] = out_[1];
  out_[1] = out_[0];
  out_[0] = (1 / (1 + c_ * c_ + M_SQRT2 * c_)) *
            (in_[2] + 2 * in_[1] + in_[0] - (c_ * c_ - M_SQRT2 * c_ + 1) * out_[2] - (-2 * c_ * c_ + 2) * out_[1]);

  if (is_debug_)
  {
    if (realtime_pub_->trylock())
    {
      realtime_pub_->msg_.header.stamp = time;
      realtime_pub_->msg_.real = in_[0];
      realtime_pub_->msg_.filtered = out_[0];
      realtime_pub_->unlockAndPublish();
    }
  }
}

void LowPassFilter::input(double in)
{
  input(in, rclcpp::Clock().now());
}

double LowPassFilter::output()
{
  return out_[0];
}

void LowPassFilter::reset()
{
  for (int i = 0; i < 3; ++i)
  {
    in_[i] = 0;
    out_[i] = 0;
  }
}
