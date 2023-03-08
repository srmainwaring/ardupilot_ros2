#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/Time.hpp"

using std::placeholders::_1;

class APTimeSubscriber : public rclcpp::Node
{
  public:
    APTimeSubscriber()
    : Node("ap_time_subscriber")
    {
      subscription_ = this->create_subscription<builtin_interfaces::msg::Time>(
      "ROS2_Time", 10, std::bind(
          &APTimeSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(
          const builtin_interfaces::msg::Time::SharedPtr msg) const
    {
      if (msg->sec) {
        RCLCPP_INFO(this->get_logger(), "From AP : True");
      } else {
        RCLCPP_INFO(this->get_logger(), "From AP : False");
      }
    }
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr
        subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APTimeSubscriber>());
  rclcpp::shutdown();
  return 0;
}