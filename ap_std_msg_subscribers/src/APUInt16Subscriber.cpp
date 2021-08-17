#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

using std::placeholders::_1;

class APUInt16Subscriber : public rclcpp::Node
{
  public:
    APUInt16Subscriber()
    : Node("ap_uint16_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
      "AP_ROS2_UInt16", 10, std::bind(&APUInt16Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt16::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APUInt16Subscriber>());
  rclcpp::shutdown();
  return 0;
}