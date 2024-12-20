#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
    // init rclcpp
    rclcpp::init(argc, argv);
    // produce node_01
    auto node = std::make_shared<rclcpp::Node>("node_01");
    // print hello
    RCLCPP_INFO(node->get_logger(), "Hello world");
    // spin and detect Ctrl-C
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;

}