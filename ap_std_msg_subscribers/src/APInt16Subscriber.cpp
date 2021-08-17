#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

using std::placeholders::_1;

class APInt16Subscriber : public rclcpp::Node
{
  public:
    APInt16Subscriber()
    : Node("ap_int16_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int16>(
      "AP_ROS2_Int16", 10, std::bind(&APInt16Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int16::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APInt16Subscriber>());
  rclcpp::shutdown();
  return 0;
}