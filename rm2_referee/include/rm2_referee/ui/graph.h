//
// Created by peter on 2021/7/19.
//

#pragma once

#include "common/data.h"
#include <rm2_common/ros_utilities.h>

namespace rm2_referee
{
class Graph
{
public:
  explicit Graph(const XmlRpc::XmlRpcValue& config, Base& base, int id);
  void updatePosition(int index);
  void setOperation(const rm2_referee::GraphOperation& operation)
  {
    config_.operate_type = operation;
  }
  int getOperation()
  {
    return config_.operate_type;
  }
  rm2_referee::GraphConfig getConfig()
  {
    return config_;
  }
  std::string getCharacters()
  {
    return title_ + content_;
  }

  void setColor(const rm2_referee::GraphColor& color)
  {
    config_.color = color;
  }
  void setContent(const std::string& content)
  {
    content_ = content;
    if (!title_.empty() || !content_.empty())
      config_.end_angle = static_cast<int>((title_ + content_).size());
  }
  void setEndX(int end_x)
  {
    config_.end_x = end_x;
  }
  void setEndY(int end_y)
  {
    config_.end_y = end_y;
  }
  void setRadius(int radius)
  {
    config_.radius = radius;
  }
  void setIntNum(int num)
  {
    int a = num & 1023;
    int b = (num >> 10) & 2047;
    int c = num >> 21;
    config_.radius = a;
    config_.end_x = b;
    config_.end_y = c;
  }
  void setFloatNum(float data)
  {
    int num = static_cast<int>(data * 1000);
    int a = num & 1023;
    int b = (num >> 10) & 2047;
    int c = num >> 21;
    config_.radius = a;
    config_.end_x = b;
    config_.end_y = c;
  }
  void setStartX(int start_x)
  {
    config_.start_x = start_x;
  }
  void setStartY(int start_y)
  {
    config_.start_y = start_y;
  }
  void setStartAngle(int start_angle)
  {
    if (0 <= start_angle && start_angle <= 360)
      config_.start_angle = start_angle;
  }
  void setEndAngle(int end_angle)
  {
    if (0 <= end_angle && end_angle <= 360)
      config_.end_angle = end_angle;
  }
  bool isRepeated()
  {
    return config_ == last_config_ && title_ == last_title_ && content_ == last_content_;
  }
  bool isString()
  {
    return config_.graphic_type == rm2_referee::GraphType::STRING;
  }
  void updateLastConfig()
  {
    last_content_ = content_;
    last_title_ = title_;
    last_config_ = config_;
  }

private:
  void initPosition(XmlRpc::XmlRpcValue value, std::vector<std::pair<int, int>>& positions);
  rm2_referee::GraphColor getColor(const std::string& color);
  rm2_referee::GraphType getType(const std::string& type);

  Base& base_;
  std::vector<std::pair<int, int>> start_positions_{}, end_positions_{};
  rm2_referee::GraphConfig config_{}, last_config_{};
  std::string title_{}, content_{}, last_title_{}, last_content_{};
};

}  // namespace rm2_referee
