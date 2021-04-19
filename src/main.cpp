#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "map_check/map_check.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapCheck>());
    rclcpp::shutdown();
    return 0;
}