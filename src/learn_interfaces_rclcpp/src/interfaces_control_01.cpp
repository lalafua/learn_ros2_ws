#include "rclcpp/rclcpp.hpp"
#include "learn_interfaces/msg/robot_status.hpp"
#include "learn_interfaces/srv/move_robot.hpp"
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>

class InterfacesControl : public rclcpp::Node{
    public:
        InterfacesControl(std::string name) : Node(name){
            RCLCPP_INFO(this->get_logger(), "Node starting: %s", name.c_str());
            move_robot_client_ = this->create_client<learn_interfaces::srv::MoveRobot>("move_robot");
            robot_status_subscribe_ = this->create_subscription<learn_interfaces::msg::RobotStatus>(
                "robot_status", 10, std::bind(&InterfacesControl::robot_status_callback_, this, std::placeholders::_1));
             
        }

        /*
        @brief request the robot to move
        first, wait for the server to start
        second, send the request

        @param float distance
        */
        void move_robot(float distance){
            RCLCPP_INFO(this->get_logger(), "Request to move robot: %f", distance);

            // wait for the server to start 
            while(!move_robot_client_->wait_for_service(std::chrono::seconds(1))){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exit...");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting...");
            }

            // send the request
            auto request = std::make_shared<learn_interfaces::srv::MoveRobot::Request>();
            request->distance = distance;

            move_robot_client_->async_send_request(
                request, std::bind(&InterfacesControl::response_callback_, this, std::placeholders::_1));   
        }

    private:
        rclcpp::Client<learn_interfaces::srv::MoveRobot>::SharedPtr move_robot_client_;
        rclcpp::Subscription<learn_interfaces::msg::RobotStatus>::SharedPtr robot_status_subscribe_;

        void response_callback_(rclcpp::Client<learn_interfaces::srv::MoveRobot>::SharedFuture result_future){
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Received move result: %f", response->pose);
        }

        /*
        @brief callback function for the robot_status topic

        @param msg
        */ 
        void robot_status_callback_(const learn_interfaces::msg::RobotStatus::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received robot position: %f status: %d", msg->pose, msg->status);
        }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfacesControl>("interfaces_control_01");
    node->move_robot(5.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;   
}   
