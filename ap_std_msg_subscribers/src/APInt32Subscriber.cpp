#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class APInt32Subscriber : public rclcpp::Node
{
  public:
    APInt32Subscriber()
    : Node("ap_int32_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "AP_ROS2_Int32", 10, std::bind(&APInt32Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%d'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APInt32Subscriber>());
  rclcpp::shutdown();
  return 0;
}