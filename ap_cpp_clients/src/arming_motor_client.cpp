#include "rclcpp/rclcpp.hpp"
#include "ap_custom_services/srv/arm_motors.hpp"        

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

char arm[] = "arm";
char disarm[] = "disarm";

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: arm_motors_client X ");     
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arm_motors_client"); // CHANGE
  rclcpp::Client<ap_custom_services::srv::ArmMotors>::SharedPtr client =                        // CHANGE
    node->create_client<ap_custom_services::srv::ArmMotors>("arm_motors");                  // CHANGE

  auto request = std::make_shared<ap_custom_services::srv::ArmMotors::Request>();               // CHANGE
  
  if(strcmp(argv[1],arm) == 0){
    request->arm = true;
  }else if(strcmp(argv[1],disarm) == 0){
    request->arm = false;
  }else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid Commands");
    return 0;
  }

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto res = client->async_send_request(request);
  // Wait for the response.
  if (rclcpp::spin_until_future_complete(node, res) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if(res.get()->result == true){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motors : Armed");
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motors : Disarmed");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service arm_motors");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}