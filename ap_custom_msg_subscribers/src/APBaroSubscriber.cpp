#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ap_custom_interfaces/msg/ap_baro.hpp"
using std::placeholders::_1;

class APBaroSubscriber : public rclcpp::Node
{
public:
    APBaroSubscriber()
    :Node("ap_baro_subscriber")
    {
        subscription_ = this->create_subscription<ap_custom_interfaces::msg::APBaro>(
        "AP_BaroTopic",10,std::bind(&APBaroSubscriber::topic_callback,this,_1));
    }
private:
    rclcpp::Subscription<ap_custom_interfaces::msg::APBaro>::SharedPtr subscription_;
    void topic_callback(const ap_custom_interfaces::msg::APBaro::SharedPtr msg) const
    {
        if (msg->healthy) {
            RCLCPP_INFO(this->get_logger(),"From AP, Health:True");
        } else {
            RCLCPP_INFO(this->get_logger(),"From AP, Health:False");
        }
        RCLCPP_INFO(this->get_logger(),"From AP, Pressure:'%f'",msg->pressure);
        RCLCPP_INFO(this->get_logger(),"From AP, Pressure Correction:'%f'",msg->pressure_correction);
        RCLCPP_INFO(this->get_logger(),"From AP, Temperature:'%f'",msg->temperature);
        RCLCPP_INFO(this->get_logger(),"From AP, Altitude:'%f'",msg->altitude);
    }
};

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<APBaroSubscriber>());
    rclcpp::shutdown();
    return 0;
}