#include "../include/scout_avoid_walls/wall_avoider.hpp"
#include <algorithm> // For std::min_element
#include <cmath>     // For std::isfinite

WallAvoider::WallAvoider() : Node("wall_avoider")
{
    this->lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scout_mini/scan", 10, std::bind(&WallAvoider::LidarCallback, this, std::placeholders::_1));

    this->keyboard_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/scout_mini/cmd_vel", 10, std::bind(&WallAvoider::KeyboardCallback, this, std::placeholders::_1));

    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/scout_mini/cmd_vel_safe", 10);

    RCLCPP_INFO(this->get_logger(), "Wall Avoidance System (WAS) is now active.");
}

// Function to safely find the minimum valid distance
float WallAvoider::SafeMin(const std::vector<float> &ranges, size_t start, size_t end)
{
    if (start >= end || start >= ranges.size() || end > ranges.size())
    {
        return std::numeric_limits<float>::infinity(); // Return a large value if range is invalid
    }

    float min_val = std::numeric_limits<float>::infinity();
    for (size_t i = start; i < end; i++)
    {
        if (std::isfinite(ranges[i]) && ranges[i] > 0.0)
        { // Ignore NaN and zero
            min_val = std::min(min_val, ranges[i]);
        }
    }
    return min_val;
}

void WallAvoider::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    geometry_msgs::msg::Twist final_cmd = this->last_key_command_;

    size_t total_points = msg->ranges.size();

    if (total_points < 4) // Avoid segmentation fault if data is too small
    {
        RCLCPP_ERROR(this->get_logger(), "Lidar data too small (%zu points), ignoring frame.", total_points);
        return;
    }

    size_t front_start = (5 * total_points) / 12;
    size_t front_end = (7 * total_points) / 12;

    size_t rear_start = 0;
    size_t rear_end = total_points / 12;
    size_t rear_start2 = (11 * total_points) / 12;
    size_t rear_end2 = total_points;

    size_t right_start = total_points / 12;
    size_t right_end = (5 * total_points) / 12;

    size_t left_start = (7 * total_points) / 12;
    size_t left_end = (11 * total_points) / 12;

    float min_front = SafeMin(msg->ranges, front_start, front_end);
    float min_right = SafeMin(msg->ranges, right_start, right_end);
    float min_rear = std::min(SafeMin(msg->ranges, rear_start, rear_end),
                              SafeMin(msg->ranges, rear_start2, rear_end2));

    float min_left = SafeMin(msg->ranges, left_start, left_end);

    RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f, Rear: %.2f",
                min_front, min_left, min_right, min_rear);

    const float threshold_slow = 1.2;
    const float threshold_stop = 0.5;

    bool is_moving_forward = this->last_key_command_.linear.x > 0.0;
    bool is_moving_backward = this->last_key_command_.linear.x < 0.0;

    // **FORWARD MOVEMENT AVOIDANCE**
    if (is_moving_forward)
    {
        if (min_front < threshold_stop)
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle too close ahead! Stopping.");
            final_cmd.linear.x = 0.0;
        }
        else if (min_front < threshold_slow)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle ahead! Slowing down.");
            final_cmd.linear.x *= 0.3;
        }

        if (min_front < threshold_slow)
        {
            if (min_right < min_left) // Obstacle is on the right
            {
                RCLCPP_INFO(this->get_logger(), "Obstacle on the right! Turning left.");
                final_cmd.angular.z = 1.0; // Turn left
            }
            else // Obstacle is on the left
            {
                RCLCPP_INFO(this->get_logger(), "Obstacle on the left! Turning right.");
                final_cmd.angular.z = -1.0; // Turn right
            }
        }
    }

    // **BACKWARD MOVEMENT AVOIDANCE**
    if (is_moving_backward && min_rear < threshold_stop)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close behind! Stopping.");
        final_cmd.linear.x = 0.0;
    }

    // **AVOID SIDE COLLISIONS DURING TURNS**
    if (final_cmd.angular.z > 0.0 && min_left < threshold_slow)
    {
        RCLCPP_WARN(this->get_logger(), "Too close to the left wall! Reducing rotation.");
        final_cmd.angular.z *= 0.5;
    }
    if (final_cmd.angular.z < 0.0 && min_right < threshold_slow)
    {
        RCLCPP_WARN(this->get_logger(), "Too close to the right wall! Reducing rotation.");
        final_cmd.angular.z *= 0.5;
    }

    this->cmd_vel_publisher_->publish(final_cmd);
}

void WallAvoider::KeyboardCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->last_key_command_ = *msg;
}

// ros2 run --prefix 'gdbserver localhost:3000' scout_avoid_walls avoid_walls