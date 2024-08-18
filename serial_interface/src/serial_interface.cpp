#include <serial_interface.hpp>

SerialInterface::SerialInterface() : Node("serial_interface_node")
    {      
        motors_array_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "motors_array", 10, std::bind(&SerialInterface::motors_array_callback, this, std::placeholders::_1));

        // write_serial();  
    }

void SerialInterface::motors_array_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 0) {
            RCLCPP_WARN(this->get_logger(), "Received motors_array is empty.");
            return;
        }

        for (int i = 0; i < 10; i++)
        {
            new_position[i+1] = msg->data[i];
        }
        
        // Check for errors
        if (serial_port < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %d opening %s: %s", errno, "/dev/ttyUSB0", strerror(errno));
            return;
        }

        // Configure serial port
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        // Read in existing settings, and handle any error
        if (tcgetattr(serial_port, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %d from tcgetattr: %s", errno, strerror(errno));
            return;
        }

        // Set Baud Rate
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        // Setting other Port Stuff
        tty.c_cflag &= ~PARENB; // Make 8n1
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag &= ~CRTSCTS; // No flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | ICRNL | IGNCR); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OCRNL;

        // Set in/out baud rate to be 115200
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %d from tcsetattr: %s", errno, strerror(errno));
            return;
        }

        // Prepare the data with start and end markers
        uint8_t start_marker = '<';
        uint8_t end_marker = '>';

        // Write the start marker
        write(serial_port, &start_marker, 1);

        // Write to serial port
        for (int i = 0; i < 11; ++i) {
            int16_t value = new_position[i];
            uint8_t bytes[2];
            bytes[0] = value & 0xFF;
            bytes[1] = (value >> 8) & 0xFF;
            write(serial_port, bytes, 2);
        }

        // Write the end marker
        write(serial_port, &end_marker, 1);
        
        return;
    }

SerialInterface::~SerialInterface(){
    close_serial();
}

void SerialInterface::close_serial() {
    // Close the serial port
    close(serial_port);
    return;
}
