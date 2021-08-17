#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ap_custom_interfaces/msg/apgps.hpp"
using std::placeholders::_1;

class APGPSSubscriber : public rclcpp::Node
{
public:
    APGPSSubscriber()
    :Node("ap_gps_subscriber")
    {
        subscription_ = this->create_subscription<ap_custom_interfaces::msg::APGPS>(
        "AP_GPSTopic",10,std::bind(&APGPSSubscriber::topic_callback,this,_1));
    }
private:
    rclcpp::Subscription<ap_custom_interfaces::msg::APGPS>::SharedPtr subscription_;
    void topic_callback(const ap_custom_interfaces::msg::APGPS::SharedPtr msg) const
    {
        if (msg->healthy) {
            RCLCPP_INFO(this->get_logger(),"From AP, Health:True");
        } else {
            RCLCPP_INFO(this->get_logger(),"From AP, Health:False");
        }
        RCLCPP_INFO(this->get_logger(),"From AP, Altitude:'%d'",msg->location_alt);
        RCLCPP_INFO(this->get_logger(),"From AP, Latitude:'%d'",msg->location_lat);
        RCLCPP_INFO(this->get_logger(),"From AP, Longitude:'%d'",msg->location_long);

        RCLCPP_INFO(this->get_logger(),"From AP, Velocity(North):'%f'",msg->velocity_n);
        RCLCPP_INFO(this->get_logger(),"From AP, Velocity(East):'%f'",msg->velocity_e);
        RCLCPP_INFO(this->get_logger(),"From AP, Velocity(Down):'%f'",msg->velocity_d);

        RCLCPP_INFO(this->get_logger(),"From AP, Ground Speed:'%d'",msg->ground_speed);
        RCLCPP_INFO(this->get_logger(),"From AP, Ground Course:'%f'",msg->ground_course);

        RCLCPP_INFO(this->get_logger(),"From AP, Number of Satellites locked:'%d'",msg->num_sat);
        RCLCPP_INFO(this->get_logger(),"From AP, Time Week:'%d'",msg->time_week);
        RCLCPP_INFO(this->get_logger(),"From AP, Time of Week:'%d'",msg->time_of_week);

        RCLCPP_INFO(this->get_logger(),"From AP, Horizontal DOP:'%d'",msg->horizontal_dop);
        RCLCPP_INFO(this->get_logger(),"From AP, Vertical DOP:'%d'",msg->vertical_dop);
    }
};

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<APGPSSubscriber>());
    rclcpp::shutdown();
    return 0;
}