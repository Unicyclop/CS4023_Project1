#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <cmath>
#include <memory>
#include <algorithm>

class OccupancyGridMapper : public rclcpp::Node
{
public:
    OccupancyGridMapper()
    //added fix for velocity time out
        : Node("occupancy_grid_mapper",
       rclcpp::NodeOptions().parameter_overrides(
           {rclcpp::Parameter("use_sim_time", true)}))
    {
        scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &OccupancyGridMapper::scanCallback,
                    this,
                    std::placeholders::_1));

        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &OccupancyGridMapper::odomCallback,
                    this,
                    std::placeholders::_1));

        map_pub_ =
            this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/map",
                10);

        map_.header.frame_id = "odom";

        map_.info.resolution = 0.05;
        map_.info.width = 300;
        map_.info.height = 200;

        map_.info.origin.position.x = -7.5;
        map_.info.origin.position.y = -5.0;
        map_.info.origin.position.z = 0.0;

        map_.info.origin.orientation.x = 0.0;
        map_.info.origin.orientation.y = 0.0;
        map_.info.origin.orientation.z = 0.0;
        map_.info.origin.orientation.w = 1.0;

        map_.data.resize(
            map_.info.width * map_.info.height,
            -1);
    }

private:

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ =
            msg->pose.pose.position.x;

        robot_y_ =
            msg->pose.pose.position.y;

        double x =
            msg->pose.pose.orientation.x;

        double y =
            msg->pose.pose.orientation.y;

        double z =
            msg->pose.pose.orientation.z;

        double w =
            msg->pose.pose.orientation.w;

        robot_yaw_ =
            std::atan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z));
    }


    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float distance = msg->ranges[i];

            if (!std::isfinite(distance))
            {
                continue;
            }

            if (distance < msg->range_min ||
                distance > msg->range_max)
            {
                continue;
            }

            double scan_angle =
                msg->angle_min +
                i * msg->angle_increment;

            double world_angle =
                robot_yaw_ +
                scan_angle;

            double obstacle_x =
                robot_x_ +
                distance * std::cos(world_angle);

            double obstacle_y =
                robot_y_ +
                distance * std::sin(world_angle);

            int robot_cell_x =
                worldToMapX(robot_x_);

            int robot_cell_y =
                worldToMapY(robot_y_);

            int obstacle_cell_x =
                worldToMapX(obstacle_x);

            int obstacle_cell_y =
                worldToMapY(obstacle_y);

            if (!validCell(
                    robot_cell_x,
                    robot_cell_y) ||
                !validCell(
                    obstacle_cell_x,
                    obstacle_cell_y))
            {
                continue;
            }

            int dx =
                obstacle_cell_x -
                robot_cell_x;

            int dy =
                obstacle_cell_y -
                robot_cell_y;

            int steps =
                std::max(
                    std::abs(dx),
                    std::abs(dy));

            if (steps > 0)
            {
                for (int step = 0; step < steps; step++)
                {
                    double fraction =
                        static_cast<double>(step) /
                        steps;

                    int cell_x =
                        robot_cell_x +
                        static_cast<int>(dx * fraction);

                    int cell_y =
                        robot_cell_y +
                        static_cast<int>(dy * fraction);

                    if (validCell(cell_x, cell_y))
                    {
                        int index =
                            cell_y * map_.info.width +
                            cell_x;

                        map_.data[index] = 0;
                    }
                }
            }

            int obstacle_index =
                obstacle_cell_y * map_.info.width +
                obstacle_cell_x;

            map_.data[obstacle_index] = 100;
        }

        map_.header.stamp =
            this->get_clock()->now();

        map_pub_->publish(map_);
    }


    int worldToMapX(double x)
    {
        return static_cast<int>(
            (x - map_.info.origin.position.x) /
            map_.info.resolution);
    }


    int worldToMapY(double y)
    {
        return static_cast<int>(
            (y - map_.info.origin.position.y) /
            map_.info.resolution);
    }


    bool validCell(int x, int y)
    {
        return x >= 0 &&
               x < static_cast<int>(map_.info.width) &&
               y >= 0 &&
               y < static_cast<int>(map_.info.height);
    }


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        map_pub_;

    nav_msgs::msg::OccupancyGrid map_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<OccupancyGridMapper>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
