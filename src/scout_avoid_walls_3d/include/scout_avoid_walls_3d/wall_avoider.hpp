#ifndef WALL_AVOIDER_3D_HPP
#define WALL_AVOIDER_3D_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

/**
 * @class WallAvoider3D
 * @brief A ROS2 node for avoiding obstacles using a 3D LIDAR and keyboard inputs.
 *
 * The WallAvoider 3D class subscribes to LIDAR and keyboard topics to control a robot's movement,
 * avoiding collisions with obstacels by adjusting its velocity commands.
 */
class WallAvoider3D : public rclcpp::Node
{
public:
    WallAvoider3D();

    /**
     * @struct MovementParams
     * @brief Parameters for movement and obstacle avoidance.
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
    struct MovementParams
    {
        float min_front;
        float min_right;
        float min_left;
        float min_rear;
        float front_obstacle_angle;
    };

private:
    double linear_speed_;
    double angular_speed_;

    double threshold_stop_;
    double threshold_slow_;

    double threshold_min_height_;
    double threshold_max_height_;

    geometry_msgs::msg::Twist last_key_command_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void KeyboardCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    MovementParams InitializeMovementParams();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void FilterGroundPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void AnalyzePointCloud(MovementParams &mov_params, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void HandleForwardMovement(MovementParams &mov_params, geometry_msgs::msg::Twist &final_cmd);
    void HandleBackwardMovement(MovementParams &mov_params, geometry_msgs::msg::Twist &final_cmd);
};

#endif // WALL_AVOIDER_HPP