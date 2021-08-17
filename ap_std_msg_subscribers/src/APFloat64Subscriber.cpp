#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

class APFloat64Subscriber : public rclcpp::Node
{
  public:
    APFloat64Subscriber()
    : Node("ap_float64_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "AP_ROS2_Float64", 10, std::bind(&APFloat64Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%f'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APFloat64Subscriber>());
  rclcpp::shutdown();
  return 0;
}