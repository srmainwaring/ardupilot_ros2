#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ap_custom_interfaces/msg/apins.hpp"
using std::placeholders::_1;

class APINSSubscriber : public rclcpp::Node
{
public:
    APINSSubscriber()
    :Node("ap_ins_subscriber")
    {
        subscription_ = this->create_subscription<ap_custom_interfaces::msg::APINS>(
        "AP_INSTopic",10,std::bind(&APINSSubscriber::topic_callback,this,_1));
    }
private:
    rclcpp::Subscription<ap_custom_interfaces::msg::APINS>::SharedPtr subscription_;
    void topic_callback(const ap_custom_interfaces::msg::APINS::SharedPtr msg) const
    {
        if (msg->gyro_healthy) {
            RCLCPP_INFO(this->get_logger(),"From AP, Gyro Health:True");
        } else {
            RCLCPP_INFO(this->get_logger(),"From AP, Gyro Health:False");
        }
        RCLCPP_INFO(this->get_logger(),"From AP, Gyro Count:'%d'",msg->gyro_count);
        RCLCPP_INFO(this->get_logger(),"From AP, Gyro(X):'%f'",msg->gyro_x);
        RCLCPP_INFO(this->get_logger(),"From AP, Gyro(Y):'%f'",msg->gyro_y);
        RCLCPP_INFO(this->get_logger(),"From AP, Gyro(Z):'%f'",msg->gyro_z);

        if (msg->accel_healthy) {
            RCLCPP_INFO(this->get_logger(),"From AP, Accel Health:True");
        } else {
            RCLCPP_INFO(this->get_logger(),"From AP, Accel Health:False");
        }
        RCLCPP_INFO(this->get_logger(),"From AP, Accel Count:'%d'",msg->accel_count);
        RCLCPP_INFO(this->get_logger(),"From AP, Accel(X):'%f'",msg->accel_x);
        RCLCPP_INFO(this->get_logger(),"From AP, Accel(Y):'%f'",msg->accel_y);
        RCLCPP_INFO(this->get_logger(),"From AP, Accel(Z):'%f'",msg->accel_z);
    }
};

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<APINSSubscriber>());
    rclcpp::shutdown();
    return 0;
}