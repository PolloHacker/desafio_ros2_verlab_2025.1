#include "../include/scout_avoid_walls_3d/wall_avoider.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallAvoider3D>());
    rclcpp::shutdown();

    return 0;
}