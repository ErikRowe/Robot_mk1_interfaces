#include <serial_interface.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialInterface>());
    rclcpp::shutdown();
    return 0;
}