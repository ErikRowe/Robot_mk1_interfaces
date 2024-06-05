#include <serial_interface.hpp>

SerialInterface::SerialInterface() : Node("serial_interface_node")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
<<<<<<< HEAD
        this->declare_parameter<int>("baudrate", 115200);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        sp_return result = sp_get_port_by_name(port.c_str(), &serial_port_);
        if (result != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find serial port: %s", port.c_str());
            return;
        }

        result = sp_open(serial_port_, SP_MODE_WRITE);
        if (result != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            return;
        }

        sp_set_baudrate(serial_port_, baudrate);
        sp_set_bits(serial_port_, 8);
        sp_set_parity(serial_port_, SP_PARITY_NONE);
        sp_set_stopbits(serial_port_, 1);
        sp_set_flowcontrol(serial_port_, SP_FLOWCONTROL_NONE);

        joystick_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&SerialInterface::joy_callback, this, std::placeholders::_1));

        //delete later
        int start_position[10]={90,180,25,155,55,
                        90,15,140,25,105};        

        int bytes_written = sp_nonblocking_write(serial_port_, start_position, 20);
        if (bytes_written != 10) {
            RCLCPP_INFO(this->get_logger(), "Failed to write data to serial port.");
        }
    }

void SerialInterface::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 10) {
            RCLCPP_WARN(this->get_logger(), "Received joystick message with less than 10 axes.");
            return;
        }

        uint8_t data[10];
        for (size_t i = 0; i < 10; ++i) {
            data[i] = static_cast<uint8_t>(msg->axes[i] * 255); // Scaling joystick value to 0-255
        }

        int bytes_written = sp_nonblocking_write(serial_port_, data, 10);
        if (bytes_written != 10) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write data to serial port.");
        }
    }
