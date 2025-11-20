#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/set_temp_threshold.hpp"
#include "example_interfaces/msg/float32.hpp"
#include <random>
#include <deque>

class TemperaturePublisherNode : public rclcpp::Node 
{
public:
    TemperaturePublisherNode() : Node("temp_publisher_node")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Float32>("temp", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TemperaturePublisherNode::tempPub_TimerCallback, this));
    }
private:
    void tempPub_TimerCallback()
    {
        auto msg = example_interfaces::msg::Float32();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(32.0f, 110.4f);

        msg.data = dist(gen);
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Temperature published: %f", msg.data);
    }
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemperaturePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}