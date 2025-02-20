#ifndef WALL_AVOIDER_HPP
#define WALL_AVOIDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class WallAvoider : public rclcpp::Node
{
public:
    WallAvoider();

private:
    struct MovementParams
    {
        float min_front;
        float min_right;
        float min_left;
        float threshold_stop;
        float threshold_slow;
    };

    geometry_msgs::msg::Twist last_key_command_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    float SafeMin(const std::vector<float> &ranges, size_t start, size_t end);
    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void KeyboardCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    std::tuple<float, float, float, float> CalculateMinDistances(const std::vector<float> &ranges, size_t total_points);
    void HandleForwardMovement(bool is_moving_forward, float min_front, float min_right, float min_left, float threshold_stop, float threshold_slow, geometry_msgs::msg::Twist &final_cmd);
    void HandleBackwardMovement(bool is_moving_backward, float min_rear, float threshold_stop, geometry_msgs::msg::Twist &final_cmd);
    void AvoidSideCollisions(geometry_msgs::msg::Twist &final_cmd, float min_left, float min_right, float threshold_slow);
};

#endif // WALL_AVOIDER_HPP