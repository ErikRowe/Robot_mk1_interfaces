
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"
#include <array>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>

class SerialInterface : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr motors_array_;

    // Port
    int serial_port = open("/dev/ttyUSB0", O_RDWR);


    // Functions
    void motors_array_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void write_serial();
    void close_serial();

    

    int start_position[11]={0,90,180,25,155,55,     // For testing  
                        90,15,140,25,105};          // For testing
    int new_position[11] = {1,90, 180-10, 25, 155, 55,
                            90, 15+10, 140, 25, 105}; 
    int set_point[11];
    
public:
    SerialInterface();
    ~SerialInterface();
};