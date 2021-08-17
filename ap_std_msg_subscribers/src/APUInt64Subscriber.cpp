#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

using std::placeholders::_1;

class APUInt64Subscriber : public rclcpp::Node
{
  public:
    APUInt64Subscriber()
    : Node("ap_uint64_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::UInt64>(
      "AP_ROS2_UInt64", 10, std::bind(&APUInt64Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::UInt64::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APUInt64Subscriber>());
  rclcpp::shutdown();
  return 0;
}