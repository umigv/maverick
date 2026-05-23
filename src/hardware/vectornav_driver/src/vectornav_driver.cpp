#include "vectornav_driver.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VectornavDriver>();

    if (!node->init()) {
        return -1;
    }

    while (rclcpp::ok()) {
        node->publish_messages();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
}
