#include <libserialport.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <array>


class SerialInterface : public rclcpp::Node
{
private:
    std::array<int,10> message_to_send;
    struct sp_port *serial_port_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscription_;


    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    std::array<int,10> calculate_engines();
    void send_serial(std::array<int,10> *message_)

    
public:
    SerialInterface();
};