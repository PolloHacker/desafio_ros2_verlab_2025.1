/**
 * @file wall_avoider.cpp
 * @brief Implementation of the WallAvoider3D class for obstacle avoidance using 3D point cloud data.
 *
 * This file contains the implementation of the WallAvoider3D class, which is responsible for
 * detecting obstacles around a robot using 3D point cloud data and adjusting the robot's movement
 * to avoid collisions. The class subscribes to point cloud and keyboard command topics, processes
 * the point cloud data to identify obstacles, and publishes safe velocity commands.
 *
 * Subscriptions:
 * - /scout_mini/scan/points: Subscribes to PointCloud2 messages for obstacle detection.
 * - /scout_mini/cmd_vel: Subscribes to Twist messages for keyboard control input.
 *
 * Publishers:
 * - /scout_mini/cmd_vel_safe: Publishes Twist messages for safe velocity commands.
 *
 * Classes:
 * - WallAvoider3D: Main class for 3D wall avoidance.
 * - MovementParams: Struct for storing movement parameters and obstacle distances.
 *
 * Functions:
 * - WallAvoider3D::WallAvoider3D(): Constructor for initializing the WallAvoider3D node.
 * - WallAvoider3D::InitializeMovementParams(): Initializes movement parameters.
 * - WallAvoider3D::ConvertPointCloud(): Converts a ROS PointCloud2 message to a PCL PointCloud.
 * - WallAvoider3D::FilterGroundPoints(): Filters out ground points from the input point cloud.
 * - WallAvoider3D::AnalyzePointCloud(): Analyzes a 3D point cloud to determine obstacle proximity.
 * - WallAvoider3D::HandleForwardMovement(): Handles forward movement based on obstacle proximity.
 * - WallAvoider3D::HandleBackwardMovement(): Handles backward movement based on obstacle proximity.
 * - WallAvoider3D::CloudCallback(): Processes point cloud messages to adjust movement commands.
 * - WallAvoider3D::KeyboardCallback(): Adjusts velocities based on keyboard commands.
 *
 * @note This implementation uses the Point Cloud Library (PCL) for point cloud processing.
 */

#include "../include/scout_avoid_walls_3d/wall_avoider.hpp"

/**
 * @struct WallAvoider3D::MovementParams
 * @brief Parameters for movement and obstacle avoidance.
 *
 * This structure holds various parameters used for detecting obstacles
 * and making movement decisions based on sensor readings.
 *
 * @var MovementParams::min_front
 * Minimum distance to an obstacle in front of the robot.
 *
 * @var MovementParams::min_right
 * Minimum distance to an obstacle on the right side of the robot.
 *
 * @var MovementParams::min_left
 * Minimum distance to an obstacle on the left side of the robot.
 *
 * @var MovementParams::min_rear
 * Minimum distance to an obstacle behind the robot.
 *
 * @var MovementParams::front_obstacle_angle
 * Angle at which an obstacle is detected in front of the robot.
 */
WallAvoider3D::MovementParams
{
    float min_front;
    float min_right;
    float min_left;
    float min_rear;
    float front_obstacle_angle;
};


/**
 * @brief Construct a new WallAvoider3D object.
 *
 * This constructor initializes the WallAvoider3D node with default parameters for linear and angular speed,
 * distance thresholds for stopping and slowing down, and height thresholds for obstacle detection.
 *
 * Subscriptions:
 * - /scout_mini/scan/points: Subscribes to PointCloud2 messages for obstacle detection.
 * - /scout_mini/cmd_vel: Subscribes to Twist messages for keyboard control input.
 *
 * Publishers:
 * - /scout_mini/cmd_vel_safe: Publishes Twist messages for safe velocity commands.
 */
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

/**
 * @brief Initializes the movement parameters for the WallAvoider3D.
 *
 * @return MovementParams A struct containing the initialized movement parameters.
 */
MovementParams WallAvoider3D::InitializeMovementParams()
{
    return {
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        0.0};
}

/**
 * @brief Converts a ROS PointCloud2 message to a PCL PointCloud.
 *
 * @param msg A shared pointer to the sensor_msgs::msg::PointCloud2 message to be converted.
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr A pointer to the converted PCL PointCloud.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr WallAvoider3D::ConvertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
}

/**
 * @brief Filters out ground points from the input point cloud based on height thresholds.
 *
 * @param cloud A pointer to the input point cloud to be filtered. The filtered point cloud
 *              will be stored in the same pointer.
 */
void WallAvoider3D::FilterGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(this->threshold_min_height_, this->threshold_max_height_);
    pass.filter(*cloud);
}

/**
 * @brief Analyzes a 3D point cloud to determine the proximity of obstacles in different directions
 * and calculates the weighted average angle of obstacles in front.
 *
 * @param mov_params A reference to a MovementParams structure that will be updated with the minimum distances and the front obstacle angle.
 * @param cloud A pointer to a pcl::PointCloud containing pcl::PointXYZ points representing the 3D point cloud data.
 */
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

/**
 * @brief Handles the forward movement of the robot by adjusting its linear and angular velocities
 *        based on the proximity of obstacles detected in front, left, and right directions.
 *
 * @param mov_params A reference to a struct containing the minimum distances to obstacles in front, left, and right,
 *                   as well as the angle to the closest front obstacle.
 * @param final_cmd A reference to a geometry_msgs::msg::Twist message that will be modified to
 *                  set the desired linear and angular velocities for the robot.
 */
void WallAvoider3D::HandleForwardMovement(MovementParams &mov_params, geometry_msgs::msg::Twist &final_cmd)
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
    }
    else
    {
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

/**
 * @brief Handles the backward movement of the robot by adjusting the speed based on the proximity of obstacles behind.
 *
 * @param mov_params A struct containing movement parameters, including the minimum distance to an obstacle behind the robot.
 * @param final_cmd A reference to a geometry_msgs::msg::Twist message that contains the final command velocities for the robot.
 *
 */
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

/**
 * @brief This function is triggered when a new point cloud message is received. It processes the point cloud
 * to determine the presence of obstacles around the robot and adjusts the movement commands to avoid collisions.
 *
 * @param msg Shared pointer to the received PointCloud2 message.
 */
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

/**
 * @brief This function is called whenever a new keyboard command is received.
 * It adjusts the linear and angular velocities based on the scout mini's real speed.
 *
 * @param msg Shared pointer to the received Twist message containing the keyboard command.
 */
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
