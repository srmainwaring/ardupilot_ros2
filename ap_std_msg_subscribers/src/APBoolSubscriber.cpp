#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class APBoolSubscriber : public rclcpp::Node
{
  public:
    APBoolSubscriber()
    : Node("ap_bool_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "AP_ROS2_Bool", 10, std::bind(&APBoolSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) const
    {
      if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "From AP : True");
      } else {
        RCLCPP_INFO(this->get_logger(), "From AP : False");
      }
    }
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APBoolSubscriber>());
  rclcpp::shutdown();
  return 0;
}