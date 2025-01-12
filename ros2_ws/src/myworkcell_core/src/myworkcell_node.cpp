#include <rclcpp/rclcpp.hpp>
#include <myworkcell_core/srv/localize_part.hpp>

class ScanNPlan : public rclcpp::Node
{
public:
  ScanNPlan() : Node("scan_n_plan") 
  {
    vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
    this->declare_parameter("base_frame", "world");
  }

  void start(const std::string& base_frame)
  {
    RCLCPP_INFO(get_logger(), "Attempting to localize part");

    // Wait for service to be available
    if (!vision_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), 
        "Unable to find localize_part service. Start vision_node first ");
        return;
    }

    // Create a request for the LocalizePart service call
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();

    auto future = vision_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) 
        != rclcpp::executor::FutureReturnCode::SUCCESS )
        {
            RCLCPP_ERROR(this->get_logger(), "LocalizePart service failed");
            return;
        }
    
    RCLCPP_INFO(get_logger(), "part localized");
    request->base_frame = base_frame;
    RCLCPP_INFO_STREAM(get_logger(), "Requesting pose in base frame: " << base_frame);

  }

private:
  //planning stuff
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;

};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto app = std::make_shared<ScanNPlan>();
    std::string base_frame = app->get_parameter("base_frame").as_string();
    // wait 2 seconds for vision node to receive data
    rclcpp::sleep_for(std::chrono::seconds(2));
    app->start(base_frame);

    rclcpp::shutdown();
    return 0;
}