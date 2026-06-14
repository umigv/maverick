#include "vectornav_driver.hpp"

int main(int argc, char* argv[]) {
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<VectornavDriver>();

        if (!node->init()) {
            return -1;
        }

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();

        rclcpp::shutdown();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("vectornav_driver"), "Unhandled exception: %s", e.what());
        return -1;
    }
}
