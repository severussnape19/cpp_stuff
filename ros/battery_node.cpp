#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/set_led.hpp"
#include "custom_interfaces/msg/led_panel_state.hpp"
#include <chrono>

class BatteryStatusNode : public rclcpp::Node
{
public:
    BatteryStatusNode() : Node("battery_node")
    {
        client_ = this->create_client<custom_interfaces::srv::SetLed>("set_led");
        is_empty_ = false; //battery
        next_duration_ = std::chrono::seconds(4);
        timer_ = this->create_wall_timer(
            next_duration_,
            std::bind(&BatteryStatusNode::timer_callback, this));
    }
private:
    void call_set_led(int led_number, int led_state)
    {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service...");
            return;
        }
        auto request = std::make_shared<custom_interfaces::srv::SetLed::Request>();
        request->led_state = led_state;
        request->led_number = led_number;

        client_->async_send_request(request);
    }
    void timer_callback()
    {
        is_empty_ = !is_empty_;
        if (is_empty_) {
            RCLCPP_INFO(this->get_logger(), "Battery Empty -> turning on LED");
            call_set_led(2, 1);
            next_duration_ = std::chrono::seconds(6);
        } else {
            RCLCPP_INFO(this->get_logger(), "Battery full -> turning LED off");
            call_set_led(2, 0);
            next_duration_ = std::chrono::seconds(4);
        }
        // timer with new duration
        timer_->cancel();
        timer_ = this->create_wall_timer(
            next_duration_,
            std::bind(&BatteryStatusNode::timer_callback, this));
    }
    rclcpp::Client<custom_interfaces::srv::SetLed>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::seconds next_duration_;
    bool is_empty_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryStatusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}