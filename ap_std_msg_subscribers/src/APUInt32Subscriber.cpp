#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

using std::placeholders::_1;

class APUInt32Subscriber : public rclcpp::Node
{
  public:
    APUInt32Subscriber()
    : Node("ap_uint32_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
      "AP_ROS2_UInt32", 10, std::bind(&APUInt32Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APUInt32Subscriber>());
  rclcpp::shutdown();
  return 0;
}