#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>

class Node03 : public rclcpp::Node{
    public:
        Node03(std::string name) : rclcpp::Node(name){
            RCLCPP_INFO(this->get_logger(), "Hello everyone,I am %s.", name.c_str());
        }
    private:
};