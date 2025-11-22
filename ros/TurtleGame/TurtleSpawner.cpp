#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "custom_interfaces/msg/alive_turtle.hpp"
#include "custom_interfaces/msg/alive_turtles.hpp"
#include "custom_interfaces/srv/catch_turtle.hpp"
#include "turtlesim/srv/kill.hpp"

class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner")
    {
        this->declare_parameter("timer", 3.0);
        timer_period_ = this->get_parameter("timer").as_double();

        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period_),
            std::bind(&TurtleSpawnerNode::spawn_turtle, this)
        );
        alive_pub_ = this->create_publisher<custom_interfaces::msg::AliveTurtles>("alive_turtles", 10);
        
        catch_server_ = this->create_service<custom_interfaces::srv::CatchTurtle>(
        "catch_turtle",
        std::bind(&TurtleSpawnerNode::catch_turtle_callback, this,
            std::placeholders::_1, std::placeholders::_2));

        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
    }
private:
    void spawn_turtle()
    {
        if (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "spawn service not available yet.");
            return;
        }

        auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
        req->x = 2.0 + (std::rand() % 7);
        req->y = 2.0 + (std::rand() % 7);
        req->theta = 0.0;

        auto future = spawn_client_->async_send_request(req,
            [this, req](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture result) {
                auto resp = result.get();

                custom_interfaces::msg::AliveTurtle t;
                t.name = resp->name;
                t.x = req->x;
                t.y = req->y;
                
                alive_turtles_.push_back(t);

                publish_alive_turtles();
            }
        );
        RCLCPP_INFO(this->get_logger(), "Requested new turtle at (%.1f, %.1f)", req->x, req->y);
    }

    void publish_alive_turtles()
    {
        custom_interfaces::msg::AliveTurtles msg;
        for (auto &t : alive_turtles_) {
            msg.turtles.push_back(t);
        }
        alive_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publish %zu alive turtles", alive_turtles_.size());
    }

    void catch_turtle_callback(
        const std::shared_ptr<custom_interfaces::srv::CatchTurtle::Request> request,
        std::shared_ptr<custom_interfaces::srv::CatchTurtle::Response> response)
    {
        std::string target = request->name;

        auto it = std::find_if(
            alive_turtles_.begin(),
            alive_turtles_.end(),
            [&](const auto& t) {return t.name == target;}
        );

        auto kill_req = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_req->name = target;

        if (!kill_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "kill service unavailable");
            response->success = false;
            return;
        }

        kill_client_->async_send_request(kill_req);

        alive_turtles_.erase(it);
        publish_alive_turtles();

        RCLCPP_INFO(this->get_logger(), "Turtle %s killed and removed!", target.c_str());
        response->success = true;
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Publisher<custom_interfaces::msg::AliveTurtles>::SharedPtr alive_pub_;
    std::vector<custom_interfaces::msg::AliveTurtle> alive_turtles_;

    rclcpp::Service<custom_interfaces::srv::CatchTurtle>::SharedPtr catch_server_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    double timer_period_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleSpawnerNode>());
    rclcpp::shutdown();
}