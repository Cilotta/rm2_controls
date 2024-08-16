//
// Created by ljq on 2022/5/17.
//

#include "rm2_referee/referee.h"

int main(int argc, char** argv)
{
  std::string robot;
  rclcpp::init(argc, argv);  // rm2_referee
  auto nh = rclcpp::Node::make_shared("rm2_referee")
  rm2_referee::Referee referee(nh);
  rclcpp::Rate loop_rate(80);
  while (rclcpp::ok())
  {
    rclcpp::spinOnce();
    referee.read();
    loop_rate.sleep();
  }

  return 0;
}
