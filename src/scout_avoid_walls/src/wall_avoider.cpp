/*
To debug this node, create a launch.json and run:
ros2 run --prefix 'gdbserver localhost:3000' scout_avoid_walls avoid_walls
*/

#include "../include/scout_avoid_walls/wall_avoider.hpp"
#include <algorithm>
#include <cmath>

WallAvoider::WallAvoider() : Node("wall_avoider"),
                             linear_speed_(3.0), angular_speed_(0.5235),
                             threshold_stop_(0.7), threshold_slow_(1.3)
{
    this->lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scout_mini/scan", 10, std::bind(&WallAvoider::LidarCallback, this, std::placeholders::_1));

    this->keyboard_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/scout_mini/cmd_vel", 10, std::bind(&WallAvoider::KeyboardCallback, this, std::placeholders::_1));

    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/scout_mini/cmd_vel_safe", 10);

    RCLCPP_INFO(this->get_logger(), "Wall Avoidance System (WAS) is now active.");
}

float WallAvoider::SafeMin(const std::vector<float> &ranges, size_t start, size_t end)
{
    if (start >= end || start >= ranges.size() || end > ranges.size())
    {
        return std::numeric_limits<float>::infinity();
    }

    float min_val = std::numeric_limits<float>::infinity();
    for (size_t i = start; i < end; i++)
    {
        if (std::isfinite(ranges[i]) && ranges[i] > 0.0)
        {
            min_val = std::min(min_val, ranges[i]);
        }
    }
    return min_val;
}

void WallAvoider::LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    geometry_msgs::msg::Twist final_cmd = this->last_key_command_;

    size_t total_points = msg->ranges.size();

    if (total_points < 12) // Avoid segmentation fault if there's not enough data
    {
        RCLCPP_ERROR(this->get_logger(), "Lidar data too small (%zu points), ignoring frame.", total_points);
        return;
    }

    MovementParams mov_params = CalculateMinDistances(msg->ranges, total_points);

    RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f, Rear: %.2f",
                mov_params.min_front, mov_params.min_left, mov_params.min_right, mov_params.min_rear);

    bool is_moving_forward = this->last_key_command_.linear.x > 0.0;
    bool is_moving_backward = this->last_key_command_.linear.x < 0.0;

    if (is_moving_forward)
    {
        HandleForwardMovement(mov_params, final_cmd);
    }
    if (is_moving_backward)
    {
        HandleBackwardMovement(mov_params, final_cmd);
    }

    final_cmd.linear.x *= this->linear_speed_;
    final_cmd.angular.z *= this->angular_speed_;

    this->cmd_vel_publisher_->publish(final_cmd);
}

WallAvoider::MovementParams WallAvoider::CalculateMinDistances(const std::vector<float> &ranges, size_t total_points)
{
    // Lidar sensor works counterclockwise starting from the X axis (where the robot is facing)

    size_t front_start = 0;
    size_t front_end = total_points * 30 / 360;
    size_t front_start2 = total_points * 330 / 360;
    size_t front_end2 = total_points;

    size_t left_start = total_points * 30 / 360;
    size_t left_end = total_points * 150 / 360;

    size_t rear_start = total_points * 150 / 360;
    size_t rear_end = total_points * 210 / 360;

    size_t right_start = total_points * 210 / 360;
    size_t right_end = total_points * 330 / 360;

    float min_front = std::min(SafeMin(ranges, front_start, front_end),
                               SafeMin(ranges, front_start2, front_end2));
    float min_right = SafeMin(ranges, right_start, right_end);
    float min_rear = SafeMin(ranges, rear_start, rear_end);
    float min_left = SafeMin(ranges, left_start, left_end);

    return {min_front, min_right, min_left, min_rear};
}

void WallAvoider::HandleForwardMovement(MovementParams mov_params, geometry_msgs::msg::Twist &final_cmd)
{
    if (mov_params.min_front < this->threshold_stop_)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close ahead! Stopping.");
        final_cmd.linear.x = 0.0;
    }
    else if (mov_params.min_front < this->threshold_slow_)
    {
        RCLCPP_INFO(this->get_logger(), "Obstacle ahead! Slowing down.");
        final_cmd.linear.x *= 0.5;
    }

    if (mov_params.min_front < this->threshold_slow_)
    {
        if (mov_params.min_right < mov_params.min_left)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle on the right! Turning left.");
            final_cmd.angular.z = 1.0;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle on the left! Turning right.");
            final_cmd.angular.z = -1.0;
        }
    }
}

void WallAvoider::HandleBackwardMovement(MovementParams mov_params, geometry_msgs::msg::Twist &final_cmd)
{
    if (mov_params.min_rear < this->threshold_slow_)
    {
        RCLCPP_INFO(this->get_logger(), "Obstacle behind! Slowing down.");
        final_cmd.linear.x *= 0.5;
    }
    if (mov_params.min_rear < this->threshold_stop_)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close behind! Stopping.");
        final_cmd.linear.x = 0.0;
        final_cmd.angular.z = 0.0;
    }
}

void WallAvoider::KeyboardCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->last_key_command_ = *msg;
}