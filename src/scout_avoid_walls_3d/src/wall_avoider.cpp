#include "../include/scout_avoid_walls_3d/wall_avoider.hpp"

WallAvoider3D::WallAvoider3D() : Node("wall_avoider_3d"),
                                 linear_speed_(3.0), angular_speed_(0.5235),
                                 threshold_stop_(0.7), threshold_slow_(1.4),
                                 threshold_min_height_(0.08), threshold_max_height_(2.0)
{
    this->cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/scout_mini/scan/points", 10, std::bind(&WallAvoider3D::CloudCallback, this, std::placeholders::_1));

    this->keyboard_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/scout_mini/cmd_vel", 10, std::bind(&WallAvoider3D::KeyboardCallback, this, std::placeholders::_1));

    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/scout_mini/cmd_vel_safe", 10);

    RCLCPP_INFO(this->get_logger(), "3D Wall Avoidance System (WAS3) is now active.");
}

WallAvoider3D::MovementParams WallAvoider3D::InitializeMovementParams()
{
    return {
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        0.0};
}

pcl::PointCloud<pcl::PointXYZ>::Ptr WallAvoider3D::ConvertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
}

void WallAvoider3D::FilterGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(this->threshold_min_height_, this->threshold_max_height_);
    pass.filter(*cloud);
}

void WallAvoider3D::AnalyzePointCloud(MovementParams &mov_params, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    float sum_weighted_angles = 0.0;
    float total_weight = 0.0;

    for (const auto &point : cloud->points)
    {
        float distance = std::sqrt(point.x * point.x + point.y * point.y);
        float angle = atan2(point.y, point.x) * 180.0 / M_PI;

        if (angle > -45 && angle < 45)
        {
            mov_params.min_front = std::min(mov_params.min_front, distance);

            float weight = 1.0 / (distance + 0.01);
            sum_weighted_angles += angle * weight;
            total_weight += weight;
        }
        else if (angle > 45 && angle < 135)
        {
            mov_params.min_left = std::min(mov_params.min_left, distance);
        }
        else if (angle < -45 && angle > -135)
        {
            mov_params.min_right = std::min(mov_params.min_right, distance);
        }
        else
        {
            mov_params.min_rear = std::min(mov_params.min_rear, distance);
        }
    }

    if (total_weight > 0.0)
    {
        mov_params.front_obstacle_angle = sum_weighted_angles / total_weight;
    }
    else
    {
        mov_params.front_obstacle_angle = 0.0;
    }
}

void WallAvoider3D::HandleForwardMovement(MovementParams mov_params, geometry_msgs::msg::Twist &final_cmd)
{

    bool is_in_corridor =
        mov_params.min_left < this->threshold_slow_ && mov_params.min_right < this->threshold_slow_;

    if (mov_params.min_front < this->threshold_stop_)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close ahead! Stopping.");
        final_cmd.linear.x = 0.0;
    }
    else if (mov_params.min_front < this->threshold_slow_)
    {
        if (!is_in_corridor)
        {
            float scale_factor = (mov_params.min_front - this->threshold_stop_) / (this->threshold_slow_ - this->threshold_stop_);
            final_cmd.linear.x *= scale_factor;
            RCLCPP_INFO(this->get_logger(), "Obstacle ahead! Slowing down to %.2f m/s.",
                        final_cmd.linear.x);
        }
    } else {
        return;
    }

    if (mov_params.front_obstacle_angle < 0)
    {
        RCLCPP_INFO(this->get_logger(), "Obstacle to the right! Turning left.");
        final_cmd.angular.z = this->angular_speed_;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Obstacle to the left! Turning right.");
        final_cmd.angular.z = -this->angular_speed_;
    }

    final_cmd.linear.x = std::min(final_cmd.linear.x, this->linear_speed_);
}

void WallAvoider3D::HandleBackwardMovement(MovementParams mov_params, geometry_msgs::msg::Twist &final_cmd)
{
    if (mov_params.min_rear < this->threshold_stop_)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close behind! Stopping.");
        final_cmd.linear.x = 0.0;
    }
    else if (mov_params.min_rear < this->threshold_slow_)
    {
        float scale_factor = (mov_params.min_rear - this->threshold_stop_) / (this->threshold_slow_ - this->threshold_stop_);
        final_cmd.linear.x *= scale_factor;
        RCLCPP_INFO(this->get_logger(), "Obstacle behind! Slowing down to %.2f m/s.",
                    final_cmd.linear.x);
    }

    final_cmd.linear.x = std::max(final_cmd.linear.x, -this->linear_speed_);
}

void WallAvoider3D::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    geometry_msgs::msg::Twist final_cmd = this->last_key_command_;
    WallAvoider3D::MovementParams mov_params = InitializeMovementParams();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = ConvertPointCloud(msg);
    FilterGroundPoints(cloud);
    AnalyzePointCloud(mov_params, cloud);

    RCLCPP_INFO(this->get_logger(),
                "Front: %.2f, Left: %.2f, Right: %.2f, Rear: %.2f, Front Angle: %.2f",
                mov_params.min_front, mov_params.min_left, mov_params.min_right,
                mov_params.min_rear, mov_params.front_obstacle_angle);

    bool is_moving_forward = this->last_key_command_.linear.x > 0.0;
    bool is_moving_backward = this->last_key_command_.linear.x < 0.0;

    if (is_moving_forward)
        HandleForwardMovement(mov_params, final_cmd);
    else if (is_moving_backward)
        HandleBackwardMovement(mov_params, final_cmd);

    this->cmd_vel_pub_->publish(final_cmd);
}

void WallAvoider3D::KeyboardCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->last_key_command_ = *msg;
    if (this->last_key_command_.linear.x < 0.0)
    {
        this->last_key_command_.linear.x = this->last_key_command_.linear.x * this->linear_speed_;
    }
    else
    {
        this->last_key_command_.linear.x = (this->last_key_command_.linear.x / 0.5) * this->linear_speed_;
    }
    this->last_key_command_.angular.z *= this->angular_speed_; // dividing by one is doing nothing
}
