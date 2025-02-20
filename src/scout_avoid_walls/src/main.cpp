#include "../include/scout_avoid_walls/wall_avoider.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallAvoider>());
    rclcpp::shutdown();

    return 0;
}