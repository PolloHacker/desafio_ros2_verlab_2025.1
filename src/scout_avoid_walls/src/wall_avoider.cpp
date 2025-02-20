/*
To debug this node, create a launch.json and run:
ros2 run --prefix 'gdbserver localhost:3000' scout_avoid_walls avoid_walls
*/

#include "../include/scout_avoid_walls/wall_avoider.hpp"
#include <algorithm>
#include <cmath>

WallAvoider::WallAvoider() : Node("wall_avoider"),
                             linear_speed_(3.0), angular_speed_(0.5235),
                             threshold_stop_(0.5), threshold_slow_(1.2)
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

    if (total_points < 4) // Avoid segmentation fault if there's not enough data
    {
        RCLCPP_ERROR(this->get_logger(), "Lidar data too small (%zu points), ignoring frame.", total_points);
        return;
    }

    MovementParams mov_params = CalculateMinDistances(msg->ranges, total_points);

    RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f, Rear: %.2f",
                mov_params.min_front, mov_params.min_left, mov_params.min_right, mov_params.min_rear);

    bool is_moving_forward = this->last_key_command_.linear.x > 0.0;
    bool is_moving_backward = this->last_key_command_.linear.x < 0.0;

    HandleForwardMovement(is_moving_forward, mov_params, final_cmd);
    HandleBackwardMovement(is_moving_backward, mov_params, final_cmd);
    AvoidSideCollisions(final_cmd, mov_params);

    this->cmd_vel_publisher_->publish(final_cmd);
}

WallAvoider::MovementParams WallAvoider::CalculateMinDistances(const std::vector<float> &ranges, size_t total_points)
{
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

    float min_front = SafeMin(ranges, front_start, front_end);
    float min_right = SafeMin(ranges, right_start, right_end);
    float min_rear = std::min(SafeMin(ranges, rear_start, rear_end),
                              SafeMin(ranges, rear_start2, rear_end2));
    float min_left = SafeMin(ranges, left_start, left_end);

    return {min_front, min_right, min_rear, min_left};
}
void WallAvoider::HandleForwardMovement(bool is_moving_forward, MovementParams mov_params, geometry_msgs::msg::Twist &final_cmd)
{
    if (is_moving_forward)
    {
        if (mov_params.min_front < this->threshold_stop_)
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle too close ahead! Stopping.");
            final_cmd.linear.x = 0.0;
        }
        else if (mov_params.min_front < this->threshold_slow_)
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle ahead! Slowing down.");
            final_cmd.linear.x *= 0.3;
        }

        if (mov_params.min_front < this->threshold_slow_)
        {
            if (mov_params.min_right < mov_params.min_left) // Obstacle is on the right
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
}

void WallAvoider::HandleBackwardMovement(bool is_moving_backward, MovementParams mov_params, geometry_msgs::msg::Twist &final_cmd)
{
    if (is_moving_backward && mov_params.min_rear < this->threshold_stop_)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close behind! Stopping.");
        final_cmd.linear.x = 0.0;
    }
}

void WallAvoider::AvoidSideCollisions(geometry_msgs::msg::Twist &final_cmd, MovementParams mov_params)
{
    if (final_cmd.angular.z > 0.0 && mov_params.min_left < this->threshold_slow_)
    {
        RCLCPP_WARN(this->get_logger(), "Too close to the left wall! Reducing rotation.");
        final_cmd.angular.z *= 0.5;
    }
    if (final_cmd.angular.z < 0.0 && mov_params.min_right < this->threshold_slow_)
    {
        RCLCPP_WARN(this->get_logger(), "Too close to the right wall! Reducing rotation.");
        final_cmd.angular.z *= 0.5;
    }
}

void WallAvoider::KeyboardCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->last_key_command_ = *msg;
}