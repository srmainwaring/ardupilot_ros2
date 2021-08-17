#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

using std::placeholders::_1;

class APInt8Subscriber : public rclcpp::Node
{
  public:
    APInt8Subscriber()
    : Node("ap_int8_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int8>(
      "AP_ROS2_Int8", 10, std::bind(&APInt8Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int8::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APInt8Subscriber>());
  rclcpp::shutdown();
  return 0;
}