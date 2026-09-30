#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"

using namespace std::chrono_literals;

class ReactiveTurtleBot : public rclcpp::Node
{
public:
    ReactiveTurtleBot()
    //Adding fix for velocity time out
        : Node("reactive_turtlebot",
           rclcpp::NodeOptions().parameter_overrides(
           {rclcpp::Parameter("use_sim_time", true)})),
          random_generator_(std::random_device{}()),
          random_turn_(-15.0, 15.0)
    {
        cmd_pub_ =
            this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/cmd_vel", 10);

        scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &ReactiveTurtleBot::scanCallback,
                    this,
                    std::placeholders::_1));

        odom_sub_ =
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &ReactiveTurtleBot::odomCallback,
                    this,
                    std::placeholders::_1));

        keyboard_sub_ =
            this->create_subscription<geometry_msgs::msg::TwistStamped>(
                "/keyboard_cmd_vel",
                10,
                std::bind(
                    &ReactiveTurtleBot::keyboardCallback,
                    this,
                    std::placeholders::_1));

        bumper_sub_ =
            this->create_subscription<ros_gz_interfaces::msg::Contacts>(
                "/bumper_contact",
                10,
                std::bind(
                    &ReactiveTurtleBot::bumperCallback,
                    this,
                    std::placeholders::_1));

        timer_ =
            this->create_wall_timer(
                100ms,
                std::bind(
                    &ReactiveTurtleBot::controlLoop,
                    this));
    }

private:

    // Robot behavior constants
    static constexpr double ONE_FOOT = 0.3048;
    static constexpr double OBSTACLE_DISTANCE = 0.3048;
    static constexpr double SYMMETRY_TOLERANCE = 0.10;

    static constexpr double FORWARD_SPEED = 0.2;
    static constexpr double TURN_SPEED = 0.6;

    static constexpr double ESCAPE_ANGLE = 175.0 * M_PI / 180.0;

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty())
        {
            return;
        }

        left_distance_ =
            std::numeric_limits<float>::infinity();

        front_distance_ =
            std::numeric_limits<float>::infinity();

        right_distance_ =
            std::numeric_limits<float>::infinity();

        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            double angle =
                msg->angle_min +
                i * msg->angle_increment;

            float distance = msg->ranges[i];

            if (!std::isfinite(distance))
            {
                continue;
            }

            if (angle >= -M_PI / 2 &&
                angle < -M_PI / 12)
            {
                if (distance < left_distance_)
                {
                    left_distance_ = distance;
                }
            }
            else if (angle >= -M_PI / 12 &&
                     angle <= M_PI / 12)
            {
                if (distance < front_distance_)
                {
                    front_distance_ = distance;
                }
            }
            else if (angle > M_PI / 12 &&
                     angle <= M_PI / 2)
            {
                if (distance < right_distance_)
                {
                    right_distance_ = distance;
                }
            }
        }
    }

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ =
            msg->pose.pose.position.x;

        current_y_ =
            msg->pose.pose.position.y;

        current_yaw_ =
            getYaw(msg->pose.pose.orientation);

        if (!position_initialized_)
        {
            start_x_ = current_x_;
            start_y_ = current_y_;

            position_initialized_ = true;
        }
    }

    void keyboardCallback(
        const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        keyboard_command_ = *msg;

        if (std::fabs(msg->twist.linear.x) > 0.01 ||
            std::fabs(msg->twist.angular.z) > 0.01)
        {
            keyboard_active_ = true;
        }
        else
        {
            keyboard_active_ = false;
        }
    }

    void bumperCallback(
        const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
    {
        if (!msg->contacts.empty())
        {
            bumper_detected_ = true;
            last_bumper_contact_time_ =
                this->get_clock()->now();

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Bumper collision detected");
        }
    }

    double getYaw(
        const geometry_msgs::msg::Quaternion & orientation)
    {
        double sin_yaw =
            2.0 *
            (orientation.w * orientation.z +
             orientation.x * orientation.y);

        double cos_yaw =
            1.0 -
            2.0 *
            (orientation.y * orientation.y +
             orientation.z * orientation.z);

        return std::atan2(sin_yaw, cos_yaw);
    }

    bool isSymmetricObstacle()
    {
        if (front_distance_ > OBSTACLE_DISTANCE)
        {
            return false;
        }

        if (!std::isfinite(left_distance_) ||
            !std::isfinite(right_distance_))
        {
            return false;
        }

        double difference =
            std::fabs(
                left_distance_ -
                right_distance_);

        return difference <= SYMMETRY_TOLERANCE;
    }

    double angleDifference(
        double current,
        double starting)
    {
        double difference =
            current - starting;

        while (difference > M_PI)
        {
            difference -= 2.0 * M_PI;
        }

        while (difference < -M_PI)
        {
            difference += 2.0 * M_PI;
        }

        return std::fabs(difference);
    }

    double distanceTraveled()
    {
        double x_difference =
            current_x_ - start_x_;

        double y_difference =
            current_y_ - start_y_;

        return std::sqrt(
            x_difference * x_difference +
            y_difference * y_difference);
    }

    void startEscape()
    {
        escaping_ = true;

        escape_start_yaw_ =
            current_yaw_;

        RCLCPP_INFO(
            this->get_logger(),
            "Starting 180 degree escape");
    }

    void startRandomTurn()
    {
        double random_degrees =
            random_turn_(random_generator_);

        random_turn_direction_ =
            (random_degrees < 0.0) ? -1.0 : 1.0;

        random_turn_angle_ =
            std::fabs(random_degrees) *
            M_PI / 180.0;

        random_turning_ = true;

        random_turn_start_yaw_ =
            current_yaw_;

        RCLCPP_INFO(
            this->get_logger(),
            "Starting random turn: %.1f degrees",
            random_degrees);
    }

    void controlLoop()
    {
        geometry_msgs::msg::TwistStamped command;

        command.header.stamp =
            this->get_clock()->now();

        /*
        * Clear the bumper state when contact messages
        * have stopped arriving.
        */
        if (bumper_detected_)
        {
            double time_since_contact =
                (this->get_clock()->now() -
                last_bumper_contact_time_).seconds();

            if (time_since_contact > 0.5)
            {
                bumper_detected_ = false;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Bumper contact cleared");
            }
        }
        /*
        * Priority 1:
        * Halt when a bumper collision is detected.
        */
        if (bumper_detected_)
        {
            command.twist.linear.x = 0.0;
            command.twist.angular.z = 0.0;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "BUMPER ACTIVE - ROBOT HALTED");

            cmd_pub_->publish(command);

            return;
        }

        if (!position_initialized_)
        {
            command.twist.linear.x = 0.0;
            command.twist.angular.z = 0.0;

            cmd_pub_->publish(command);

            return;
        }

	RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "LiDAR | LEFT: %.2f m | FRONT: %.2f m | RIGHT: %.2f m",
            left_distance_,
            front_distance_,
            right_distance_);

	RCLCPP_INFO_THROTTLE(
    	   this->get_logger(),
           *this->get_clock(),
    	   1000,
    	   "ODOM | X: %.2f | Y: %.2f | DISTANCE: %.2f m",
    	   current_x_,
    	   current_y_,
    	   distanceTraveled());

        /*
         * Priority 2:
         * Human keyboard commands.
         */
        if (keyboard_active_)
        {
            command.twist.linear.x =
                keyboard_command_.twist.linear.x;

            command.twist.angular.z =
                keyboard_command_.twist.angular.z;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Keyboard control active");
        }

        /*
         * Priority 3:
         * Continue an escape that has already started.
         */
        else if (escaping_)
        {
            double turned =
                angleDifference(
                    current_yaw_,
                    escape_start_yaw_);

            if (turned < ESCAPE_ANGLE)
            {
                command.twist.linear.x = 0.0;
                command.twist.angular.z = TURN_SPEED;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Escaping symmetric obstacle");
            }
            else
            {
                escaping_ = false;

                start_x_ = current_x_;
                start_y_ = current_y_;

                command.twist.linear.x =
                    FORWARD_SPEED;

                command.twist.angular.z = 0.0;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Escape complete");
            }
        }

        /*
         * Priority 3:
         * Detect a new symmetric obstacle.
         */
        else if (isSymmetricObstacle())
        {
            startEscape();

            command.twist.linear.x = 0.0;
            command.twist.angular.z = TURN_SPEED;
        }

        /*
         * Priority 4:
         * Avoid an asymmetric obstacle.
         */
        else if (front_distance_ <= OBSTACLE_DISTANCE ||
                 left_distance_ <= OBSTACLE_DISTANCE ||
                 right_distance_ <= OBSTACLE_DISTANCE)
        {
            command.twist.linear.x = 0.0;

            if (left_distance_ < right_distance_)
            {
                command.twist.angular.z = -TURN_SPEED;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Obstacle detected closer on LEFT -> commanding RIGHT turn");
            }
            else
            {
                command.twist.angular.z = TURN_SPEED;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Obstacle detected closer on RIGHT -> commanding LEFT turn");
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Avoiding asymmetric obstacle");
        }

        /*
         * Priority 5:
         * Continue a random turn already in progress.
         */
        else if (random_turning_)
        {
            double turned =
                angleDifference(
                    current_yaw_,
                    random_turn_start_yaw_);

            if (turned < random_turn_angle_)
            {
                command.twist.linear.x = 0.0;

                command.twist.angular.z =
                    TURN_SPEED *
                    random_turn_direction_;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Performing random turn");
            }
            else
            {
                random_turning_ = false;

                start_x_ = current_x_;
                start_y_ = current_y_;

                command.twist.linear.x =
                    FORWARD_SPEED;

                command.twist.angular.z = 0.0;
            }
        }

        /*
         * Priority 5:
         * After traveling one foot, start a random turn.
         */
        else if (distanceTraveled() >= ONE_FOOT)
        {
            startRandomTurn();

            command.twist.linear.x = 0.0;

            command.twist.angular.z =
                TURN_SPEED *
                random_turn_direction_;
        }

        /*
         * Priority 6:
         * Default behavior is to drive forward.
         */
        else
        {
            command.twist.linear.x =
                FORWARD_SPEED;

            command.twist.angular.z = 0.0;
        }

        cmd_pub_->publish(command);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        cmd_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        keyboard_sub_;

    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr
        bumper_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TwistStamped keyboard_command_;

    float left_distance_ =
        std::numeric_limits<float>::infinity();

    float front_distance_ =
        std::numeric_limits<float>::infinity();

    float right_distance_ =
        std::numeric_limits<float>::infinity();

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    double start_x_ = 0.0;
    double start_y_ = 0.0;

    double escape_start_yaw_ = 0.0;
    double random_turn_start_yaw_ = 0.0;
    double random_turn_angle_ = 0.0;

    double random_turn_direction_ = 1.0;

    rclcpp::Time last_bumper_contact_time_{0, 0, RCL_ROS_TIME};
    
    bool position_initialized_ = false;
    bool bumper_detected_ = false;
    bool keyboard_active_ = false;
    bool escaping_ = false;
    bool random_turning_ = false;

    std::mt19937 random_generator_;

    std::uniform_real_distribution<double> random_turn_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ReactiveTurtleBot>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
