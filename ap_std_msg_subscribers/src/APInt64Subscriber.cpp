#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using std::placeholders::_1;

class APInt64Subscriber : public rclcpp::Node
{
  public:
    APInt64Subscriber()
    : Node("ap_int64_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "AP_ROS2_Int64", 10, std::bind(&APInt64Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APInt64Subscriber>());
  rclcpp::shutdown();
  return 0;
}