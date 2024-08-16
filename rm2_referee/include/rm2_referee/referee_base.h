//
// Created by ljq on 2021/12/3.
//

#pragma once

#include <rm2_common/ros_utilities.h>
#include <rclcpp/timer.hpp>
#include <rm2_common/decision/command_sender.h>

#include "ui/ui_base.h"
#include "ui/trigger_change_ui.h"
#include "ui/time_change_ui.h"
#include "ui/flash_ui.h"
#include "ui/interactive_data.h"

namespace rm2_referee
{
class RefereeBase
{
public:
  explicit RefereeBase(rclcpp::Node& nh, Base& base);
  virtual void addUi();
  // unpack call back
  virtual void robotStatusDataCallBack(const rm2_msgs::msg::GameRobotStatus& game_robot_status_data,
                                       const rclcpp::Time& last_get_data_time);
  virtual void updateEnemyHeroState(const rm2_msgs::msg::GameRobotHp& game_robot_hp_data, const rclcpp::Time& last_get_data_time);
  virtual void gameStatusDataCallBack(const rm2_msgs::msg::GameStatus& game_status_data, const rclcpp::Time& last_get_data_time);
  virtual void capacityDataCallBack(const rm2_msgs::msg::PowerManagementSampleAndStatusData& data,
                                    rclcpp::Time& last_get_data_time);
  virtual void powerHeatDataCallBack(const rm2_msgs::msg::PowerHeatData& power_heat_data, const rclcpp::Time& last_get_data_time);
  virtual void robotHurtDataCallBack(const rm2_msgs::msg::RobotHurt& robot_hurt_data, const rclcpp::Time& last_get_data_time);
  virtual void bulletRemainDataCallBack(const rm2_msgs::msg::BulletAllowance& bullet_allowance,
                                        const rclcpp::Time& last_get_data_time);
  virtual void interactiveDataCallBack(const rm2_referee::InteractiveData& interactive_data,
                                       const rclcpp::Time& last_get_data_time);
  virtual void eventDataCallBack(const rm2_msgs::msg::EventData& event_data, const rclcpp::Time& last_get_data_time);
  virtual void updateHeroHitDataCallBack(const rm2_msgs::msg::GameRobotHp& game_robot_hp_data);
  virtual void supplyBulletDataCallBack(const rm2_msgs::msg::SupplyProjectileAction& data);
  virtual void updateShootDataDataCallBack(const rm2_msgs::msg::ShootData& msg);
  virtual void updateBulletRemainData(const rm2_referee::BulletNumData& data);

  // sub call back
  virtual void jointStateCallback(const sensor_msgs::msg::JointState::ConstPtr& joint_state);
  virtual void actuatorStateCallback(const rm2_msgs::msg::ActuatorState::ConstPtr& data);
  virtual void dbusDataCallback(const rm2_msgs::msg::DbusData::ConstPtr& data);
  virtual void chassisCmdDataCallback(const rm2_msgs::msg::ChassisCmd::ConstPtr& data);
  virtual void vel2DCmdDataCallback(const geometry_msgs::msg::Twist::ConstPtr& data);
  virtual void shootStateCallback(const rm2_msgs::msg::ShootState::ConstPtr& data);
  virtual void gimbalCmdDataCallback(const rm2_msgs::msg::GimbalCmd::ConstPtr& data);
  virtual void cardCmdDataCallback(const rm2_msgs::msg::StateCmd::ConstPtr& data);
  virtual void engineerUiDataCallback(const rm2_msgs::msg::EngineerUi::ConstPtr& data);
  virtual void manualDataCallBack(const rm2_msgs::msg::ManualToReferee::ConstPtr& data);
  virtual void radarDataCallBack(const std_msgs::msg::Int8MultiArray::ConstPtr& data);
  virtual void cameraNameCallBack(const std_msgs::String& data);
  virtual void trackCallBack(const rm2_msgs::msg::TrackData::ConstPtr& data);
  virtual void balanceStateCallback(const rm2_msgs::msg::BalanceState::ConstPtr& data);
  virtual void radarReceiveCallback(const rm2_msgs::msg::ClientMapReceiveData::ConstPtr& data);
  virtual void mapSentryCallback(const rm2_msgs::msg::MapSentryData::ConstPtr& data);
  virtual void sentryAttackingTargetCallback(const rm2_msgs::msg::SentryAttackingTarget::ConstPtr& data);
  virtual void sendSentryCmdCallback(const rm2_msgs::msg::SentryInfo::ConstPtr& data);
  virtual void sendRadarCmdCallback(const rm2_msgs::msg::RadarInfo::ConstPtr& data);
  virtual void sendSentryStateCallback(const std_msgs::String::ConstPtr& data);
  virtual void dronePoseCallBack(const geometry_msgs::msg::PoseStamped::ConstPtr& data);
  virtual void shootCmdCallBack(const rm2_msgs::msg::ShootCmd::ConstPtr& data);

  // send  ui
  void sendSerialDataCallback();
  void sendQueue();
  
  SubscriptionEntry joint_state_sub_;
  ros::Subscriber actuator_state_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber chassis_cmd_sub_;
  ros::Subscriber vel2D_cmd_sub_;
  ros::Subscriber shoot_state_sub_;
  ros::Subscriber gimbal_cmd_sub_;
  ros::Subscriber detection_status_sub_;
  ros::Subscriber card_cmd_sub_;
  ros::Subscriber calibration_status_sub_;
  ros::Subscriber engineer_cmd_sub_;
  ros::Subscriber radar_date_sub_;
  ros::Subscriber manual_data_sub_;
  ros::Subscriber camera_name_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber balance_state_sub_;
  ros::Subscriber radar_receive_sub_;
  ros::Subscriber map_sentry_sub_;
  ros::Subscriber sentry_to_radar_sub_;
  ros::Subscriber radar_to_sentry_sub_;
  ros::Subscriber sentry_cmd_sub_;
  ros::Subscriber radar_cmd_sub_;
  ros::Subscriber sentry_state_sub_;
  ros::Subscriber drone_pose_sub_;
  ros::Subscriber shoot_cmd_sub_;

  ChassisTriggerChangeUi* chassis_trigger_change_ui_{};
  ShooterTriggerChangeUi* shooter_trigger_change_ui_{};
  GimbalTriggerChangeUi* gimbal_trigger_change_ui_{};
  TargetTriggerChangeUi* target_trigger_change_ui_{};
  TargetViewAngleTriggerChangeUi* target_view_angle_trigger_change_ui_{};
  CameraTriggerChangeUi* camera_trigger_change_ui_{};
  FrictionSpeedTriggerChangeUi* friction_speed_trigger_change_ui_{};

  BulletTimeChangeUi* bullet_time_change_ui_{};
  CapacitorTimeChangeUi* capacitor_time_change_ui_{};
  EffortTimeChangeUi* effort_time_change_ui_{};
  ProgressTimeChangeUi* progress_time_change_ui_{};
  DartStatusTimeChangeUi* dart_status_time_change_ui_{};
  RotationTimeChangeUi* rotation_time_change_ui_{};
  LaneLineTimeChangeGroupUi* lane_line_time_change_ui_{};
  BalancePitchTimeChangeGroupUi* balance_pitch_time_change_group_ui_{};
  PitchAngleTimeChangeUi* pitch_angle_time_change_ui_{};
  JointPositionTimeChangeUi *engineer_joint1_time_change_ui{}, *engineer_joint2_time_change_ui{},
      *engineer_joint3_time_change_ui{};
  TargetDistanceTimeChangeUi* target_distance_time_change_ui_{};
  FriendBulletsTimeChangeGroupUi* friend_bullets_time_change_group_ui_{};

  DroneTowardsTimeChangeGroupUi* drone_towards_time_change_group_ui_{};
  StringTriggerChangeUi *servo_mode_trigger_change_ui_{}, *stone_num_trigger_change_ui_{},
      *joint_temperature_trigger_change_ui_{}, *gripper_state_trigger_change_ui_{};

  FixedUi* fixed_ui_{};

  CoverFlashUi* cover_flash_ui_{};
  SpinFlashUi* spin_flash_ui_{};
  HeroHitFlashUi* hero_hit_flash_ui_{};
  ExceedBulletSpeedFlashUi* exceed_bullet_speed_flash_ui_{};
  EngineerActionFlashUi* engineer_action_flash_ui_{};

  InteractiveSender* interactive_data_sender_{};
  CustomInfoSender* enemy_hero_state_sender_{};
  CustomInfoSender* sentry_state_sender_{};
  BulletNumShare* bullet_num_share_{};
  SentryToRadar* sentry_to_radar_{};

  GroupUiBase* graph_queue_sender_{};
  std::deque<Graph> graph_queue_;
  std::deque<Graph> character_queue_;
  //  std::deque<std::tuple<>> interactive_data_queue_;

  rclcpp::Time radar_interactive_data_last_send_;
  rclcpp::Time sentry_interactive_data_last_send_;
  rclcpp::Time sentry_cmd_data_last_send_, radar_cmd_data_last_send_;

  Base& base_;
  ros::Timer add_ui_timer_, send_serial_data_timer_;
  int add_ui_times_, add_ui_max_times_, add_ui_frequency_;
  double send_ui_queue_delay_;
  bool add_ui_flag_ = false, is_adding_ = false;
  ros::NodeHandle nh_;
  std::string dbus_topic_;
};
}  // namespace rm2_referee
