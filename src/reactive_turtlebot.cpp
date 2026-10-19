#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
// remove: #include "ros_gz_interfaces/msg/contacts.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"

using namespace std::chrono_literals;

class ReactiveTurtleBot : public rclcpp::Node
{
public:
    ReactiveTurtleBot()
    //Adding fix for velocity time out
        : Node("reactive_turtlebot"),
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

        hazard_sub_ =
            this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
                "/hazard_detection",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &ReactiveTurtleBot::hazardCallback,
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
    //static constexpr double OBSTACLE_DISTANCE = 0.3048;
    // NEW: scan is measured from the LiDAR, not the robot's front edge.
    // Measure the real distance from the LiDAR center to the front of the robot.
    static constexpr double LIDAR_TO_FRONT = 0.35;
    static constexpr double OBSTACLE_DISTANCE = ONE_FOOT + LIDAR_TO_FRONT;   // CHANGED
    static constexpr double SYMMETRY_TOLERANCE = 0.10;

    static constexpr double FORWARD_SPEED = 0.2;
    static constexpr double TURN_SPEED = 0.6;

    static constexpr double ESCAPE_ANGLE = 175.0 * M_PI / 180.0;

    // Front cone must cover the robot's width (~0.17 m each side at ~0.3-0.5 m)
    static constexpr double FRONT_HALF_ANGLE = 30.0 * M_PI / 180.0;

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty())
        {
            return;
        }

        scan_received_ = true;
        front_scan_valid_ = false;
        last_scan_time_ = std::chrono::steady_clock::now();

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
                angle < -FRONT_HALF_ANGLE)                 // CHANGED
            {
                if (distance < right_distance_)
                {
                    right_distance_ = distance;
                }
            }
            else if (angle >= -FRONT_HALF_ANGLE &&         // CHANGED
                     angle <= FRONT_HALF_ANGLE)            // CHANGED
            {
                if (distance >= msg->range_min &&
                    distance <= msg->range_max)
                {
                    front_scan_valid_ = true;

                    if (distance < front_distance_)
                    {
                        front_distance_ = distance;
                    }
                }
            }
            else if (angle > FRONT_HALF_ANGLE &&           // CHANGED
                     angle <= M_PI / 2)
            {
                if (distance < left_distance_)
                {
                    left_distance_ = distance;
                }
            }
        }

                // TEMPORARY DEBUG: where is the nearest return in the full scan?
        float nearest = std::numeric_limits<float>::infinity();
        double nearest_angle = 0.0;
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float d = msg->ranges[i];
            if (std::isfinite(d) && d >= msg->range_min && d < nearest)
            {
                nearest = d;
                nearest_angle = msg->angle_min + i * msg->angle_increment;
            }
        }
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "SCAN DEBUG | nearest %.2f m at %.0f deg | angle_min %.2f angle_max %.2f",
            nearest, nearest_angle * 180.0 / M_PI,
            msg->angle_min, msg->angle_max);

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

        void hazardCallback(
        const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg)
    {
        for (const auto & detection : msg->detections)
        {
            // NEW: the base is refusing to reverse any further
            if (detection.type ==
                irobot_create_msgs::msg::HazardDetection::BACKUP_LIMIT)
            {
                backup_limit_hit_ = true;
            }

            if (detection.type ==
                irobot_create_msgs::msg::HazardDetection::BUMP)
            {
                bumper_detected_ = true;
                last_bumper_contact_time_ =
                    this->get_clock()->now();

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Bumper collision detected (%s)",
                    detection.header.frame_id.c_str());
            }
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

        escape_turned_ = 0.0;               // NEW
        escape_last_yaw_ = current_yaw_;    // NEW

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
        /*adding back in command.header.stamp to test*/
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

                backing_up_ = true;
                backup_start_x_ = current_x_;
                backup_start_y_ = current_y_;

                backup_start_time_ = this->get_clock()->now();   // NEW
                backup_limit_hit_ = false;                       // NEW

                RCLCPP_INFO(
                    this->get_logger(),
                    "Bumper contact cleared - backing up");
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

        if (backing_up_)
        {
            double dx = current_x_ - backup_start_x_;
            double dy = current_y_ - backup_start_y_;

            double backup_distance =
                std::sqrt(dx * dx + dy * dy);

            // NEW: how long this backup has been running
            double backup_elapsed =
                (this->get_clock()->now() - backup_start_time_).seconds();

            // CHANGED: also stop on timeout or backup limit
            if (backup_distance < 0.15 &&
                backup_elapsed < 3.0 &&
                !backup_limit_hit_)
            {
                command.twist.linear.x = -0.10;
                command.twist.angular.z = 0.0;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Backing away from bumper collision");

                cmd_pub_->publish(command);
                return;
            }
            else
            {
                backing_up_ = false;
                backup_limit_hit_ = false;   // NEW

                start_x_ = current_x_;
                start_y_ = current_y_;

                // CHANGED: log why it finished
                RCLCPP_INFO(
                    this->get_logger(),
                    "Backup finished (distance %.2f m, %.1f s)",
                    backup_distance,
                    backup_elapsed);

                    startEscape();   // NEW: turn away from the wall after backing up
            }
        }

        if (!position_initialized_)
        {
            command.twist.linear.x = 0.0;
            command.twist.angular.z = 0.0;

            cmd_pub_->publish(command);

            return;
        }

        bool scan_fresh =
            scan_received_ &&
            (std::chrono::steady_clock::now() - last_scan_time_
                < std::chrono::milliseconds(500));

        if (!scan_fresh || !front_scan_valid_)
        {
            command.twist.linear.x = 0.0;
            command.twist.angular.z = 0.0;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "LiDAR data missing, stale, or invalid - stopping");

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
            // NEW: add up the yaw change each tick so passing 180 deg can't
            // be missed and wrap back around to 0
            double yaw_step = current_yaw_ - escape_last_yaw_;
            while (yaw_step > M_PI)  { yaw_step -= 2.0 * M_PI; }
            while (yaw_step < -M_PI) { yaw_step += 2.0 * M_PI; }
            escape_turned_ += std::fabs(yaw_step);
            escape_last_yaw_ = current_yaw_;

            // NEW: if the front is still blocked after the turn, keep turning
            // (at most one full rotation) instead of driving into the wall
            bool still_blocked = (front_distance_ <= OBSTACLE_DISTANCE);

            if (escape_turned_ < ESCAPE_ANGLE ||                      // CHANGED
                (still_blocked && escape_turned_ < 2.0 * M_PI))       // NEW
            {
                command.twist.linear.x = 0.0;
                command.twist.angular.z = TURN_SPEED;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Escaping obstacle (turned %.0f deg)",
                    escape_turned_ * 180.0 / M_PI);
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
            * If avoidance lasts too long, perform the existing escape turn.
            */
            else if (front_distance_ <= OBSTACLE_DISTANCE)
            {
                if (!avoiding_obstacle_)
                {
                    avoiding_obstacle_ = true;
                    obstacle_avoidance_start_time_ =
                        this->get_clock()->now();
                }

                double avoidance_time =
                    (this->get_clock()->now() -
                    obstacle_avoidance_start_time_).seconds();

                if (avoidance_time >= 2.0)
                {
                    avoiding_obstacle_ = false;

                    RCLCPP_WARN(
                        this->get_logger(),
                        "Obstacle avoidance stuck - starting escape");

                    startEscape();

                    command.twist.linear.x = 0.0;
                    command.twist.angular.z = TURN_SPEED;
                }
                else
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
            }

        /*
         * Priority 5:
         * Continue a random turn already in progress.
         */
        else if (random_turning_)
        {
            avoiding_obstacle_ = false;

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

                RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 200,
            "DBG front=%.2f thresh=%.2f esc=%d avoid=%d | cmd lin=%.2f ang=%.2f",
            front_distance_, OBSTACLE_DISTANCE,
            escaping_, avoiding_obstacle_,
            command.twist.linear.x, command.twist.angular.z);

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

    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr
        hazard_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TwistStamped keyboard_command_;

    float left_distance_ =
        std::numeric_limits<float>::infinity();

    float front_distance_ =
        std::numeric_limits<float>::infinity();

    float right_distance_ =
        std::numeric_limits<float>::infinity();

    bool scan_received_ = false;
    bool front_scan_valid_ = false;

    std::chrono::steady_clock::time_point last_scan_time_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    double start_x_ = 0.0;
    double start_y_ = 0.0;

    double escape_start_yaw_ = 0.0;

        
    double escape_turned_ = 0.0;      // NEW: total angle turned during the escape
    double escape_last_yaw_ = 0.0;    // NEW: yaw at the previous tick

    double random_turn_start_yaw_ = 0.0;
    double random_turn_angle_ = 0.0;

    double random_turn_direction_ = 1.0;

    rclcpp::Time last_bumper_contact_time_{0, 0, RCL_ROS_TIME};
    
    bool position_initialized_ = false;
    bool bumper_detected_ = false;
    bool backing_up_ = false;
    bool avoiding_obstacle_ = false;
    bool keyboard_active_ = false;
    bool escaping_ = false;
    bool random_turning_ = false;

    rclcpp::Time obstacle_avoidance_start_time_{0, 0, RCL_ROS_TIME};

    double backup_start_x_ = 0.0;
    double backup_start_y_ = 0.0;

    // NEW: backup timeout and backup-limit tracking
    rclcpp::Time backup_start_time_{0, 0, RCL_ROS_TIME};
    bool backup_limit_hit_ = false;
    

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