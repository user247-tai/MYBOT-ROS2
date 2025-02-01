#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class ConvertNode : public rclcpp::Node // MODIFY NAME
{
public:
    ConvertNode() : Node("mybot_convert") // MODIFY NAME
    {
        convert_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&ConvertNode::convert_callback, this, std::placeholders::_1)
        );
        convert_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mybot_controller/cmd_vel", 10);    
    }

private:
    void convert_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped twist_stamped;
        twist_stamped.header.stamp = get_clock()->now();
        twist_stamped.twist = *msg;
        convert_pub_->publish(twist_stamped);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr convert_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr convert_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConvertNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}