//
// Created by gura on 24-5-29.
//

#include "rm2_referee/ui/interactive_data.h"

namespace rm2_referee
{
void InteractiveSender::sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data)
{
  uint8_t tx_data[sizeof(InteractiveData)] = { 0 };
  auto student_interactive_data = (InteractiveData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  student_interactive_data->header_data.data_cmd_id = data_cmd_id;
  student_interactive_data->header_data.sender_id = base_.robot_id_;
  student_interactive_data->header_data.receiver_id = receiver_id;
  student_interactive_data->data = data;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(InteractiveData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(InteractiveData) + k_tail_length_);

  sendSerial(rclcpp::Time::now(), sizeof(InteractiveData));
}

bool InteractiveSender::needSendInteractiveData()
{
  return rclcpp::Time::now() - last_send_time_ > delay_;
}

void InteractiveSender::sendMapSentryData(const rm2_referee::MapSentryData& data)
{
  uint8_t tx_data[sizeof(rm2_referee::MapSentryData)] = { 0 };
  auto map_sentry_data = (rm2_referee::MapSentryData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  map_sentry_data->intention = data.intention;
  map_sentry_data->start_position_x = data.start_position_x;
  map_sentry_data->start_position_y = data.start_position_y;
  for (int i = 0; i < 49; i++)
  {
    map_sentry_data->delta_x[i] = data.delta_x[i];
    map_sentry_data->delta_y[i] = data.delta_y[i];
  }
  map_sentry_data->sender_id = base_.robot_id_;
  pack(tx_buffer_, tx_data, rm2_referee::RefereeCmdId::MAP_SENTRY_CMD, sizeof(rm2_referee::MapSentryData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(rm2_referee::MapSentryData) + k_tail_length_);

  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  clearTxBuffer();
}

void CustomInfoSender::sendCustomInfoData(std::wstring data)
{
  if (data == last_custom_info_ || rclcpp::Time::now() - last_send_ < ros::Duration(0.35))
    return;
  else
    last_custom_info_ = data;

  int data_len;
  rm2_referee::CustomInfo tx_data;
  data_len = static_cast<int>(sizeof(rm2_referee::CustomInfo));

  tx_data.sender_id = base_.robot_id_;
  tx_data.receiver_id = base_.client_id_;

  uint16_t characters[15];
  for (int i = 0; i < 15; i++)
  {
    if (i < static_cast<int>(data.size()))
      characters[i] = static_cast<uint16_t>(data[i]);
    else
      characters[i] = static_cast<uint16_t>(L' ');
  }
  for (int i = 0; i < 15; i++)
  {
    tx_data.user_data[2 * i] = characters[i] & 0xFF;
    tx_data.user_data[2 * i + 1] = (characters[i] >> 8) & 0xFF;
  }
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm2_referee::RefereeCmdId::CUSTOM_INFO_CMD, data_len);
  last_send_ = rclcpp::Time::now();
  sendSerial(rclcpp::Time::now(), data_len);
}

void InteractiveSender::sendRadarInteractiveData(const rm2_referee::ClientMapReceiveData& data)
{
  uint8_t tx_data[sizeof(rm2_referee::ClientMapReceiveData)] = { 0 };
  auto radar_interactive_data = (rm2_referee::ClientMapReceiveData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  radar_interactive_data->target_robot_ID = data.target_robot_ID;
  radar_interactive_data->target_position_x = data.target_position_x;
  radar_interactive_data->target_position_y = data.target_position_y;
  pack(tx_buffer_, tx_data, rm2_referee::RefereeCmdId::CLIENT_MAP_CMD, sizeof(rm2_referee::ClientMapReceiveData));
  tx_len_ =
      k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(rm2_referee::ClientMapReceiveData) + k_tail_length_);
  try
  {
    base_.serial_.write(tx_buffer_, tx_len_);
  }
  catch (serial::PortNotOpenedException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  clearTxBuffer();
}

void InteractiveSender::sendSentryCmdData(const rm2_msgs::SentryInfoConstPtr& data)
{
  int data_len;
  rm2_referee::SentryInfo tx_data;
  data_len = static_cast<int>(sizeof(rm2_referee::SentryInfo));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = REFEREE_SERVER;
  tx_data.sentry_info = data->sentry_info;

  tx_data.header.data_cmd_id = rm2_referee::DataCmdId::SENTRY_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm2_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(rclcpp::Time::now(), data_len);
}

void InteractiveSender::sendRadarCmdData(const rm2_msgs::RadarInfoConstPtr& data)
{
  int data_len;
  rm2_referee::RadarInfo tx_data;
  data_len = static_cast<int>(sizeof(rm2_referee::RadarInfo));

  tx_data.header.sender_id = base_.robot_id_;
  tx_data.header.receiver_id = REFEREE_SERVER;
  tx_data.radar_info = data->radar_info;

  tx_data.header.data_cmd_id = rm2_referee::DataCmdId::RADAR_CMD;
  pack(tx_buffer_, reinterpret_cast<uint8_t*>(&tx_data), rm2_referee::RefereeCmdId::INTERACTIVE_DATA_CMD, data_len);
  sendSerial(rclcpp::Time::now(), data_len);
}

void BulletNumShare::sendBulletData()
{
  uint16_t receiver_id;
  if (base_.robot_color_ == "red")
    receiver_id = RED_HERO;
  else
    receiver_id = BLUE_HERO;
  receiver_id += (4 - (count_receive_time_ % 3));

  uint8_t tx_data[sizeof(BulletNumData)] = { 0 };
  auto bullet_num_data = (BulletNumData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  bullet_num_data->header_data.data_cmd_id = rm2_referee::DataCmdId::BULLET_NUM_SHARE_CMD;
  bullet_num_data->header_data.sender_id = base_.robot_id_;
  bullet_num_data->header_data.receiver_id = receiver_id;
  bullet_num_data->bullet_42_mm_num = bullet_42_mm_num_;
  bullet_num_data->bullet_17_mm_num = bullet_17_mm_num_;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(BulletNumData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(BulletNumData) + k_tail_length_);
  sendSerial(rclcpp::Time::now(), sizeof(BulletNumData));
  last_send_time_ = rclcpp::Time::now();
  count_receive_time_++;
}

void BulletNumShare::updateBulletRemainData(const rm2_msgs::BulletAllowance& data)
{
  if (data.bullet_allowance_num_42_mm > 5096 || data.bullet_allowance_num_17_mm > 5096)
    return;
  bullet_17_mm_num_ = data.bullet_allowance_num_17_mm;
  bullet_42_mm_num_ = data.bullet_allowance_num_42_mm;
}

void SentryToRadar::updateSentryAttackingTargetData(const rm2_msgs::SentryAttackingTargetConstPtr& data)
{
  robot_id_ = data->target_robot_ID;
  target_position_x_ = data->target_position_x;
  target_position_y_ = data->target_position_y;
  last_get_data_time_ = rclcpp::Time::now();
}

void SentryToRadar::sendSentryToRadarData()
{
  uint8_t tx_data[sizeof(SentryAttackingTargetData)] = { 0 };
  auto sentry_attacking_target_data = (SentryAttackingTargetData*)tx_data;

  for (int i = 0; i < 127; i++)
    tx_buffer_[i] = 0;
  sentry_attacking_target_data->header_data.data_cmd_id = rm2_referee::DataCmdId::SENTRY_TO_RADAR_CMD;
  sentry_attacking_target_data->header_data.sender_id = base_.robot_id_;
  if (base_.robot_color_ == "red")
    sentry_attacking_target_data->header_data.receiver_id = RED_RADAR;
  else
    sentry_attacking_target_data->header_data.receiver_id = BLUE_RADAR;
  sentry_attacking_target_data->target_robot_ID = robot_id_;
  sentry_attacking_target_data->target_position_x = target_position_x_;
  sentry_attacking_target_data->target_position_y = target_position_y_;
  pack(tx_buffer_, tx_data, RefereeCmdId::INTERACTIVE_DATA_CMD, sizeof(SentryAttackingTargetData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(SentryAttackingTargetData) + k_tail_length_);
  sendSerial(rclcpp::Time::now(), sizeof(SentryAttackingTargetData));
  last_send_time_ = rclcpp::Time::now();
}

bool SentryToRadar::needSendInteractiveData()
{
  return InteractiveSender::needSendInteractiveData() && rclcpp::Time::now() - last_get_data_time_ < ros::Duration(0.5);
}
}  // namespace rm2_referee
