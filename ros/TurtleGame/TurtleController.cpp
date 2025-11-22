#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "custom_interfaces/msg/alive_turtles.hpp"
#include "custom_interfaces/msg/alive_turtle.hpp"
#include "custom_interfaces/srv/catch_turtle.hpp"
#include <cmath>

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        this->declare_parameter("linear_gain", 2.0);
        this->declare_parameter("angular_gain", 1.5);
        this->declare_parameter("catch_distance", 0.15);

        linear_gain_ = this->get_parameter("linear_gain").as_double();
        angular_gain_ = this->get_parameter("angular_gain").as_double();
        catch_distance_ = this->get_parameter("catch_distance").as_double();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);

        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleControllerNode::poseCallback, this, std::placeholders::_1));

        alive_sub_ = this->create_subscription<custom_interfaces::msg::AliveTurtles>(
            "/alive_turtles", 10,
            std::bind(&TurtleControllerNode::alive_callback, this, std::placeholders::_1));

        catch_client_ = this->create_client<custom_interfaces::srv::CatchTurtle>(
            "/catch_turtle");
    }

private:
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_theta_ = msg->theta;

        if (!has_target_) return;

        float dx = target_x_ - msg->x;
        float dy = target_y_ - msg->y;

        float target_theta = std::atan2(dy, dx);
        float angle_diff = target_theta - msg->theta;

        // Normalize angle
        while (angle_diff > M_PI) angle_diff -= 2*M_PI;
        while (angle_diff < -M_PI) angle_diff += 2*M_PI;

        float distance = std::sqrt(dx*dx + dy*dy);

        geometry_msgs::msg::Twist cmd;

        // Rotate first
        if (std::abs(angle_diff) > 0.2) {
            cmd.angular.z = angular_gain_ * angle_diff;
        }
        else {
            cmd.linear.x = linear_gain_ * distance;
        }
        
        if (distance < catch_distance_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;

            auto req = std::make_shared<custom_interfaces::srv::CatchTurtle::Request>();
            req->name = target_name_;

            if (catch_client_->wait_for_service(std::chrono::seconds(1))) {
                catch_client_->async_send_request(req);
                RCLCPP_INFO(this->get_logger(), "Caught turtle: %s",
                            target_name_.c_str());
            }

            has_target_ = false;
        }

        publisher_->publish(cmd);
    }

    void alive_callback(const custom_interfaces::msg::AliveTurtles::SharedPtr msg)
    {
        alive_turtles_ = msg->turtles;

        if (alive_turtles_.empty()) {
            has_target_ = false;
            return;
        }

        // picks closest turtle
        float best_dist = 1e6;
        custom_interfaces::msg::AliveTurtle best;

        for (auto &t : alive_turtles_) {
            float dx = t.x - current_x_;
            float dy = t.y - current_y_;
            float dist = std::sqrt(dx*dx + dy*dy);

            if (dist < best_dist) {
                best_dist = dist;
                best = t;
            }
        }

        target_name_ = best.name;
        target_x_ = best.x;
        target_y_ = best.y;
        has_target_ = true;

        RCLCPP_INFO(this->get_logger(), "New target: %s (%.1f, %.1f)",
                    target_name_.c_str(), target_x_, target_y_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::AliveTurtles>::SharedPtr alive_sub_;
    rclcpp::Client<custom_interfaces::srv::CatchTurtle>::SharedPtr catch_client_;

    std::vector<custom_interfaces::msg::AliveTurtle> alive_turtles_;
    bool has_target_ = false;
    std::string target_name_;
    float target_x_, target_y_;
    float current_x_ = 0, current_y_ = 0, current_theta_ = 0;

    double linear_gain_, angular_gain_, catch_distance_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControllerNode>());
    rclcpp::shutdown();
}
