#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ReactiveTurtleBot : public rclcpp::Node
{
public:
    ReactiveTurtleBot()
        : Node("reactive_turtlebot")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(
                &ReactiveTurtleBot::controlLoop,
                this));
    }

private:

    void controlLoop()
    {
        geometry_msgs::msg::Twist command;

        command.linear.x = 0.2;
        command.linear.y = 0.0;
        command.linear.z = 0.0;

        command.angular.x = 0.0;
        command.angular.y = 0.0;
        command.angular.z = 0.0;

        cmd_pub_->publish(command);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ReactiveTurtleBot>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
