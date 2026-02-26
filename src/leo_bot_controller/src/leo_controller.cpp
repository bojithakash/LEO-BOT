#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "leo_bot_msgs/msg/leo_velocity.hpp"

class LeoController : public rclcpp::Node
{
public:
    LeoController() : Node("leo_controller")
    {
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&LeoController::cmdCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&LeoController::imuCallback, this, std::placeholders::_1));

        vel_pub_ = this->create_publisher<leo_bot_msgs::msg::LeoVelocity>(
            "leo_velocity", 10);

        RCLCPP_INFO(this->get_logger(), "Leo Controller Started");
    }

private:

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        leo_bot_msgs::msg::LeoVelocity vel;
        vel.linear = msg->linear.x;
        vel.angular = msg->angular.z;
        vel_pub_->publish(vel);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // For now just log orientation
        RCLCPP_INFO(this->get_logger(),
                    "IMU Orientation Z: %.3f",
                    msg->orientation.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<leo_bot_msgs::msg::LeoVelocity>::SharedPtr vel_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeoController>());
    rclcpp::shutdown();
    return 0;
}