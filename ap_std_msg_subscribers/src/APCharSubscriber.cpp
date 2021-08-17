#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"

using std::placeholders::_1;

class APCharSubscriber : public rclcpp::Node
{
  public:
    APCharSubscriber()
    : Node("ap_char_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Char>(
      "AP_ROS2_Char", 10, std::bind(&APCharSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Char::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%c'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APCharSubscriber>());
  rclcpp::shutdown();
  return 0;
}