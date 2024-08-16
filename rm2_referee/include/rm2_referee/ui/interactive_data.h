//
// Created by gura on 24-5-29.
//

#pragma once

#include "ui_base.h"

namespace rm2_referee
{
class InteractiveSender : public UiBase
{
public:
  explicit InteractiveSender(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                             std::deque<Graph>* character_queue = nullptr)
    : UiBase(rpc_value, base, graph_queue, character_queue){};

  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);
  void sendRadarInteractiveData(const rm2_referee::ClientMapReceiveData& data);
  void sendMapSentryData(const rm2_referee::MapSentryData& data);
  void sendSentryCmdData(const rm2_msgs::SentryInfoConstPtr& data);
  void sendRadarCmdData(const rm2_msgs::RadarInfoConstPtr& data);
  virtual bool needSendInteractiveData();
  rclcpp::Time last_send_time_;
};

class CustomInfoSender : public InteractiveSender
{
public:
  explicit CustomInfoSender(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                            std::deque<Graph>* character_queue = nullptr)
    : InteractiveSender(rpc_value, base, graph_queue, character_queue){};
  void sendCustomInfoData(std::wstring data);

protected:
  std::wstring last_custom_info_;
};

class BulletNumShare : public InteractiveSender
{
public:
  explicit BulletNumShare(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                          std::deque<Graph>* character_queue = nullptr)
    : InteractiveSender(rpc_value, base, graph_queue, character_queue){};
  void sendBulletData();
  void updateBulletRemainData(const rm2_msgs::BulletAllowance& data);
  int bullet_42_mm_num_, bullet_17_mm_num_, count_receive_time_;
};

class SentryToRadar : public InteractiveSender
{
public:
  explicit SentryToRadar(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                         std::deque<Graph>* character_queue = nullptr)
    : InteractiveSender(rpc_value, base, graph_queue, character_queue){};
  void updateSentryAttackingTargetData(const rm2_msgs::SentryAttackingTargetConstPtr& data);
  bool needSendInteractiveData() override;
  void sendSentryToRadarData();
  rclcpp::Time last_get_data_time_;
  float target_position_x_, target_position_y_, robot_id_;
};

}  // namespace rm2_referee
