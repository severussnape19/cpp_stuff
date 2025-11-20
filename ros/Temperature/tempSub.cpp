#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/set_temp_threshold.hpp"
#include "example_interfaces/msg/float32.hpp"
#include <deque>

class TemperatureSubscriberNode : public rclcpp::Node
{
public:
    TemperatureSubscriberNode() : Node("temp_sub_node"), threshold_(55.0f), window_size_(5)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Float32>(
            "temp",
            10, 
            std::bind(&TemperatureSubscriberNode::callback_tempSubscriber, this, std::placeholders::_1));
        
        server_ = this->create_service<custom_interfaces::srv::SetTempThreshold>(
            "set_threshold",
            std::bind(&TemperatureSubscriberNode::callback_thresholdService, this, 
                std::placeholders::_1, std::placeholders::_2));
    }
private:
    void callback_tempSubscriber(const std::shared_ptr<example_interfaces::msg::Float32> msg)
    {
        float_t rolling_temp_ = 0.0f;
        temps_.push_back(msg->data);
        
        if (temps_.size() > window_size_) {
            temps_.pop_front();
        }
        for (auto i : temps_) {
            rolling_temp_ += i;
        }
        rolling_temp_ /= window_size_;

        if (rolling_temp_ > threshold_ * 1.15f)
        {
            RCLCPP_FATAL(this->get_logger(), "Temperature: %f", rolling_temp_);
        } else if (rolling_temp_ > threshold_) {
            RCLCPP_WARN(this->get_logger(), "Temperature: %f", rolling_temp_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Temperature: %f", rolling_temp_);
        }
    }

    void callback_thresholdService(
        const std::shared_ptr<custom_interfaces::srv::SetTempThreshold::Request> request_,
        std::shared_ptr<custom_interfaces::srv::SetTempThreshold::Response> response_) 
    {
        threshold_ = request_->threshold;
        response_->success = true;
        RCLCPP_INFO(this->get_logger(), "Threshold set to %f", threshold_);
    }

    rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr subscriber_;
    rclcpp::Service<custom_interfaces::srv::SetTempThreshold>::SharedPtr server_;
    float_t threshold_;
    std::deque<float> temps_;
    size_t window_size_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemperatureSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}