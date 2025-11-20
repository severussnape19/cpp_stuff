#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/led_panel_state.hpp"
#include "custom_interfaces/srv/set_led.hpp"
#include <vector>

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_node")
    {
        leds_ = {0, 0, 0};
        publisher_ = this->create_publisher<custom_interfaces::msg::LedPanelState>("led_panel_state", 10);
        service_ =  this->create_service<custom_interfaces::srv::SetLed>(
            "set_led",
            std::bind(&LedPanelNode::callback_service, this, 
                std::placeholders::_1, std::placeholders::_2));
        publisherMessage();
        RCLCPP_INFO(this->get_logger(), "Server and publisher active");
    }
private:
    void publisherMessage()
    {
        auto msg = custom_interfaces::msg::LedPanelState();
        msg.leds = leds_;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "[%d %d %d]", leds_[0], leds_[1], leds_[2]);
    }

    void callback_service(
        const custom_interfaces::srv::SetLed::Request::SharedPtr request,
        custom_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        int led_idx = request->led_number;
        int state = request->led_state;

        leds_[led_idx] = state;

        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Led %d set to %d", led_idx, state);

        publisherMessage();
    }
    rclcpp::Publisher<custom_interfaces::msg::LedPanelState>::SharedPtr publisher_;
    rclcpp::Service<custom_interfaces::srv::SetLed>::SharedPtr service_;
    std::vector<int32_t> leds_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}