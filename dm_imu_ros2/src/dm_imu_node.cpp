#include "rclcpp/rclcpp.hpp"
#include "dm_imu/imu_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dmbot_serial::DmImu>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
