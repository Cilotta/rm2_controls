//
// Created by llljjjqqq on 22-11-4.
//
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <rm2_common/ori_tool.h>
#include <rm2_common/decision/heat_limit.h>
#include <rm2_msgs/srv/StatusChange.srv>

#include "ui/graph.h"

namespace rm2_referee
{
class UiBase
{
public:
  explicit UiBase(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                  std::deque<Graph>* character_queue = nullptr)
    : base_(base), tf_listener_(tf_buffer_)
  {
    if (rpc_value.hasMember("config"))
      if (rpc_value["config"].hasMember("delay"))
        delay_ = ros::Duration(static_cast<double>(rpc_value["config"]["delay"]));
    graph_queue_ = graph_queue;
    character_queue_ = character_queue;
  };
  ~UiBase() = default;
  virtual void add();
  virtual void update();
  virtual void updateForQueue();
  virtual void erasure();
  virtual void addForQueue(int add_times = 1);
  virtual void updateManualCmdData(const rm2_msgs::srv::ManualToReferee::ConstPtr data){};
  virtual void updateManualCmdData(const rm2_msgs::srv::ManualToReferee::ConstPtr data, const rclcpp::Time& last_get_data_time){};
  virtual void sendUi(const rclcpp::Time& time);

  void sendCharacter(const rclcpp::Time& time, Graph* graph);
  void sendSingleGraph(const rclcpp::Time& time, Graph* graph);

  void sendSerial(const rclcpp::Time& time, int data_len);
  void clearTxBuffer();

  virtual void display(bool check_repeat = true);
  virtual void displayTwice(bool check_repeat = true);
  virtual void display(const rclcpp::Time& time);
  void display(const rclcpp::Time& time, bool state, bool once = false);
  void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len) const;

  uint8_t tx_buffer_[127];
  int tx_len_;

protected:
  Base& base_;
  Graph* graph_;
  static int id_;
  std::deque<Graph>* graph_queue_;
  std::deque<Graph>* character_queue_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Time last_send_;
  ros::Duration delay_ = ros::Duration(0.);
  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
};

class GroupUiBase : public UiBase
{
public:
  explicit GroupUiBase(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                       std::deque<Graph>* character_queue = nullptr)
    : UiBase(rpc_value, base, graph_queue, character_queue){};
  ~GroupUiBase() = default;
  void add() override;
  void update() override;
  void updateForQueue() override;
  void erasure() override;
  void addForQueue(int add_times = 1) override;
  void sendUi(const rclcpp::Time& time) override;
  void sendDoubleGraph(const rclcpp::Time& time, Graph* graph0, Graph* graph1);
  void sendFiveGraph(const rclcpp::Time& time, Graph* graph0, Graph* graph1, Graph* graph2, Graph* graph3, Graph* graph4);
  void sendSevenGraph(const rclcpp::Time& time, Graph* graph0, Graph* graph1, Graph* graph2, Graph* graph3, Graph* graph4,
                      Graph* graph5, Graph* graph6);
  void display(bool check_repeat = true) override;
  void display(const rclcpp::Time& time) override;
  void displayTwice(bool check_repeat = true) override;

protected:
  std::map<std::string, Graph*> graph_vector_;
  std::map<std::string, Graph*> character_vector_;
};

class FixedUi : public GroupUiBase
{
public:
  explicit FixedUi(XmlRpc::XmlRpcValue& rpc_value, Base& base, std::deque<Graph>* graph_queue = nullptr,
                   std::deque<Graph>* character_queue = nullptr)
    : GroupUiBase(rpc_value, base, graph_queue, character_queue)
  {
    for (int i = 0; i < static_cast<int>(rpc_value.size()); i++)
    {
      if (rpc_value[i]["config"]["type"] == "string")
      {
        ROS_INFO_STREAM("string FixedUi:" << rpc_value[i]["name"]);
        character_vector_.insert(
            std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], base_, id_++)));
      }
      else
        graph_vector_.insert(
            std::pair<std::string, Graph*>(rpc_value[i]["name"], new Graph(rpc_value[i]["config"], base_, id_++)));
    }
  };
  void updateForQueue() override;
  int update_fixed_ui_times = 0;
};

}  // namespace rm2_referee
