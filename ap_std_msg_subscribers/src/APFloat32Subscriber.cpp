#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class APFloat32Subscriber : public rclcpp::Node
{
  public:
    APFloat32Subscriber()
    : Node("ap_float32_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "AP_ROS2_Float32", 10, std::bind(&APFloat32Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "From AP : '%f'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<APFloat32Subscriber>());
  rclcpp::shutdown();
  return 0;
}