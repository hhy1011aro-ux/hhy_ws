#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "perception_storage_ros2/perception_storage.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto perception_ptr=std::make_shared<PerceptionStorage>();
  rclcpp::spin(perception_ptr);
  rclcpp::shutdown();
  return 0;
}
