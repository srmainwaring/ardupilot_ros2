#include "rclcpp/rclcpp.hpp"
#include "ap_custom_services/srv/mode_select.hpp"        

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

// enum Modes
enum DriveModes {
    Auto,
    Autotune,
    Circle,
    Guided,
    Land,
    Loiter,
    RTL,
    SmartRTL
};

// copter modes (as of now)
char mode_auto[] = "auto";
char mode_autotune[] = "autotune";
char mode_circle[] = "circle";
char mode_guided[] = "guided";
char mode_land[] = "land";
char mode_loiter[] = "loiter";
char mode_RTL[] = "rtl";
char mode_smartRTL[] = "smartrtl";

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: mode_select_client X ");     
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mode_select_client"); // CHANGE
  rclcpp::Client<ap_custom_services::srv::ModeSelect>::SharedPtr client =                        // CHANGE
    node->create_client<ap_custom_services::srv::ModeSelect>("mode_select");                  // CHANGE

  auto request = std::make_shared<ap_custom_services::srv::ModeSelect::Request>();               // CHANGE
  
  if(strcmp(argv[1],mode_auto) == 0){
    request->mode = Auto;
  }else if(strcmp(argv[1],mode_autotune) == 0){
    request->mode = Autotune;
  }else if(strcmp(argv[1],mode_circle) == 0){
    request->mode = Circle;
  }else if(strcmp(argv[1],mode_guided) == 0){
    request->mode = Guided;
  }else if(strcmp(argv[1],mode_land) == 0){
    request->mode = Land;
  }else if(strcmp(argv[1],mode_loiter) == 0){
    request->mode = Loiter;
  }else if(strcmp(argv[1],mode_RTL) == 0){
    request->mode = RTL;
  }else if(strcmp(argv[1],mode_smartRTL) == 0){
    request->mode = SmartRTL;
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
    if(res.get()->status == true){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode Switch : Success");
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode Switch : Failed");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service mode_select");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}