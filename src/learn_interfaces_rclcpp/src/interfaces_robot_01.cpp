#include "learn_interfaces/msg/robot_status.hpp"
#include "learn_interfaces/srv/move_robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>

class Robot {
    public:
        Robot() = default;
        ~Robot() = default;
        /**
        * @brief 移动指定的距离
        *
        * @param distance
        * @return float
        */
        float move_distance(float distance) {
            status_ = learn_interfaces::msg::RobotStatus::STATUS_MOVING;
            target_pose_ += distance;
            // 当目标距离和当前距离大于0.01则持续向目标移动
            while (fabs(target_pose_ - current_pose_) > 0.01) {
                // 每一步移动当前到目标距离的1/10
                float step = distance / fabs(distance) * fabs(target_pose_ - current_pose_) * 0.1;
                current_pose_ += step;
                std::cout << "Has moved:" << step << "Current position:" << current_pose_ << std::endl;
                // 当前线程休眠500ms
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        status_ = learn_interfaces::msg::RobotStatus::STATUS_STOP;
        return current_pose_;
        }       
    /**
    * @brief Get the current pose
    *
    * @return float
    */
    float get_current_pose() { return current_pose_; }

    /**
    * @brief Get the status
    *
    * @return int
    *  1 learn_interfaces::msg::RobotStatus::STATUS_MOVEING
    *  2 learn_interfaces::msg::RobotStatus::STATUS_STOP
    */
    int get_status() { 
        return status_; 
    }

    private:
        // 声明当前位置
        float current_pose_ = 0.0;
        // 目标距离
        float target_pose_ = 0.0;
        int status_ = learn_interfaces::msg::RobotStatus::STATUS_STOP;
};


class InterfacesRobot : public rclcpp::Node {
    public:
        InterfacesRobot(std::string name) : Node(name){
            RCLCPP_INFO(this->get_logger(), "Node started: %s", name.c_str());
            
            // create a service named move_robot
            move_robot_server_ = this->create_service<learn_interfaces::srv::MoveRobot>(
                "move_robot", std::bind(&InterfacesRobot::handle_move_robot, this, std::placeholders::_1, std::placeholders::_2));
            
            // create a publisher
            robot_status_publisher_ = this->create_publisher<learn_interfaces::msg::RobotStatus>("robot_status", 10);
        
            // create a timer to publish the robot status
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&InterfacesRobot::timer_callback, this));
        }
    
    private:
        Robot robot;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<learn_interfaces::srv::MoveRobot>::SharedPtr move_robot_server_;
        rclcpp::Publisher<learn_interfaces::msg::RobotStatus>::SharedPtr robot_status_publisher_;

        /*
        @brief 500ms callback function
        */
        void timer_callback(){
            learn_interfaces::msg::RobotStatus message;
            message.status = robot.get_status();
            message.pose = robot.get_current_pose();
            RCLCPP_INFO(this->get_logger(), "Publishing:%f", message.pose);
            robot_status_publisher_->publish(message);
        }

        /*
        @brief callback when receiving a message from topic
        
        @param request
        @param response
        */
        void handle_move_robot(const std::shared_ptr<learn_interfaces::srv::MoveRobot::Request> request,
                               std::shared_ptr<learn_interfaces::srv::MoveRobot::Response> response){
            RCLCPP_INFO(this->get_logger(), "Request received: %f, current position: %f", request->distance, robot.get_current_pose());
            robot.move_distance(request->distance);
            response->pose = robot.get_current_pose();
                               }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InterfacesRobot>("interfaces_robot_01");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}