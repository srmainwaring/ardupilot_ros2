#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ap_custom_interfaces/msg/ap_num.hpp"
using std::placeholders::_1;

class APNumSubscriber : public rclcpp::Node
{
public:
    APNumSubscriber()
    :Node("ap_num_subscriber")
    {
        subscription_ = this->create_subscription<ap_custom_interfaces::msg::APNum>(
        "AP_NumTopic",10,std::bind(&APNumSubscriber::topic_callback,this,_1));
    }
private:
    rclcpp::Subscription<ap_custom_interfaces::msg::APNum>::SharedPtr subscription_;
    void topic_callback(const ap_custom_interfaces::msg::APNum::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(),"From AP, Data:'%d'",msg->num);
    }
};

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<APNumSubscriber>());
    rclcpp::shutdown();
    return 0;
}