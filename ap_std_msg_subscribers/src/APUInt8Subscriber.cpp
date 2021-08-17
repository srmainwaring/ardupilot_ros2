#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

using std::placeholders::_1;

class APUInt8Subscriber : public rclcpp::Node
{
  public:
    APUInt8Subscriber()
    : Node("ap_uint8_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
      "AP_ROS2_UInt8", 10, std::bind(&APUInt8Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt8::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APUInt8Subscriber>());
  rclcpp::shutdown();
  return 0;
}