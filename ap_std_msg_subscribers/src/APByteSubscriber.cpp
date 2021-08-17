#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"

using std::placeholders::_1;

class APByteSubscriber : public rclcpp::Node
{
  public:
    APByteSubscriber()
    : Node("ap_byte_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Byte>(
      "AP_ROS2_Byte", 10, std::bind(&APByteSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Byte::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APByteSubscriber>());
  rclcpp::shutdown();
  return 0;
}