#include <memory>
#include <rclcpp/rclcpp.hpp>

class TemplateCpp : public rclcpp::Node {
  public:
    TemplateCpp() : Node("template_cpp") {}
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateCpp>());
    rclcpp::shutdown();
    return 0;
}
