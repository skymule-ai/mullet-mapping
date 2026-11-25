#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "map_localizer/map_localizer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map_localizer::MapLocalizer>());
  rclcpp::shutdown();
  return 0;
}
