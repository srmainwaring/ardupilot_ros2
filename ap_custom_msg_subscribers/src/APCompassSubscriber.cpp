#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ap_custom_interfaces/msg/ap_compass.hpp"
using std::placeholders::_1;

class APCompassSubscriber : public rclcpp::Node
{
public:
    APCompassSubscriber()
    :Node("ap_compass_subscriber")
    {
        subscription_ = this->create_subscription<ap_custom_interfaces::msg::APCompass>(
        "AP_CompassTopic",10,std::bind(&APCompassSubscriber::topic_callback,this,_1));
    }
private:
    rclcpp::Subscription<ap_custom_interfaces::msg::APCompass>::SharedPtr subscription_;
    void topic_callback(const ap_custom_interfaces::msg::APCompass::SharedPtr msg) const
    {
        if (msg->healthy) {
            RCLCPP_INFO(this->get_logger(),"From AP, Health:True");
        } else {
            RCLCPP_INFO(this->get_logger(),"From AP, Health:False");
        }
        RCLCPP_INFO(this->get_logger(),"From AP, Magnetic Field(X):'%f'",msg->magfield_x);
        RCLCPP_INFO(this->get_logger(),"From AP, Magnetic Field(Y):'%f'",msg->magfield_y);
        RCLCPP_INFO(this->get_logger(),"From AP, Magnetic Field(Z):'%f'",msg->magfield_z);
    }
};

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<APCompassSubscriber>());
    rclcpp::shutdown();
    return 0;
}