//
// Created by llljjjqqq on 22-11-4.
//

#pragma once

#include "ui_base.h"

namespace rm2_referee
{
class FlashUi : public UiBase
{
public:
  explicit FlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name,
                   std::deque<Graph>* graph_queue, std::deque<Graph>* character_queue)
    : UiBase(rpc_value, base, graph_queue, character_queue)
  {
    graph_ = new Graph(rpc_value["config"], base_, id_++);
  }
  virtual void display(const rclcpp::Time& time){};
  virtual void updateConfig(){};
  void updateFlashUiForQueue(const rclcpp::Time& time, bool state, bool once);
};

class FlashGroupUi : public GroupUiBase
{
public:
  explicit FlashGroupUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, const std::string& graph_name,
                        std::deque<Graph>* graph_queue, std::deque<Graph>* character_queue)
    : GroupUiBase(rpc_value, base, graph_queue, character_queue)
  {
    graph_name_ = graph_name;
  }
  virtual void display(const rclcpp::Time& time){};
  virtual void updateConfig(){};
  void updateFlashUiForQueue(const rclcpp::Time& time, bool state, bool once, Graph* graph);

protected:
  std::string graph_name_;
};

class EngineerActionFlashUi : public FlashGroupUi
{
public:
  explicit EngineerActionFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                 std::deque<Graph>* character_queue)
    : FlashGroupUi(rpc_value, base, "engineer_action", graph_queue, character_queue)
  {
    if (rpc_value.hasMember("data"))
    {
      XmlRpc::XmlRpcValue& data = rpc_value["data"];
      for (int i = 0; i < static_cast<int>(rpc_value["data"].size()); i++)
      {
        graph_vector_.insert(std::pair<std::string, Graph*>(std::to_string(static_cast<int>(data[i]["flag"])),
                                                            new Graph(data[i]["config"], base_, id_++)));
      }
    }
  }
  void updateEngineerUiCmdData(const rm2_msgs::EngineerUi::ConstPtr data, const rclcpp::Time& last_get_data_time);

private:
  void display(const rclcpp::Time& time) override;
  uint32_t symbol_;
};

class CoverFlashUi : public FlashUi
{
public:
  explicit CoverFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                        std::deque<Graph>* character_queue)
    : FlashUi(rpc_value, base, "cover", graph_queue, character_queue){};
  void updateManualCmdData(const rm2_msgs::ManualToReferee::ConstPtr data, const rclcpp::Time& last_get_data_time) override;

private:
  void display(const rclcpp::Time& time) override;

  uint8_t cover_state_;
};

class SpinFlashUi : public FlashUi
{
public:
  explicit SpinFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                       std::deque<Graph>* character_queue)
    : FlashUi(rpc_value, base, "spin", graph_queue, character_queue){};
  void updateChassisCmdData(const rm2_msgs::ChassisCmd::ConstPtr data, const rclcpp::Time& last_get_data_time);

private:
  void display(const rclcpp::Time& time) override;
  uint8_t chassis_mode_;
};

class HeroHitFlashUi : public FlashUi
{
public:
  explicit HeroHitFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                          std::deque<Graph>* character_queue)
    : FlashUi(rpc_value, base, " hero_hit", graph_queue, character_queue)
  {
  }
  void updateHittingConfig(const rm2_msgs::GameRobotHp& msg);

private:
  void display(const rclcpp::Time& time) override;
  bool hitted_;
  rm2_msgs::GameRobotHp last_hp_msg_;
};

class ExceedBulletSpeedFlashUi : public FlashUi
{
public:
  explicit ExceedBulletSpeedFlashUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue,
                                    std::deque<Graph>* character_queue)
    : FlashUi(rpc_value, base, "exceed_bullet_speed", graph_queue, character_queue)
  {
  }
  void updateShootData(const rm2_msgs::ShootData& msg);

private:
  void display(const rclcpp::Time& time) override;
  rm2_msgs::ShootData shoot_data_;
};

}  // namespace rm2_referee
