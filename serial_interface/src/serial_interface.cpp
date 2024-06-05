#include <serial_interface.hpp>

SerialInterface::SerialInterface() : Node("serial_interface_node")
    {      
        joystick_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&SerialInterface::joy_callback, this, std::placeholders::_1));

        write_serial();  
    }

void SerialInterface::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Received joystick message with less than 10 axes.");
            return;
        }

        uint8_t data[10];
        for (size_t i = 0; i < 10; ++i) {
            data[i] = static_cast<uint8_t>(msg->axes[i] * 255); // Scaling joystick value to 0-255
        }

        // RCLCPP_INFO(this->get_logger(), "pos is %d ", data[0]);
        // if (data[0] == 1.0){
        //     set_point[11] = {1,90, 180-10, 25, 155, 55,
        //                     90, 15+10, 140, 25, 105}; 
        // } else if (data[0] == -1.0) {
        //     start_position[11]={0,90,180,25,155,55,     // For testing  
        //                 90,15,140,25,105}; 
        // }

    }

void SerialInterface::write_serial() {
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

    // Write to serial port
    for (int i = 0; i < 11; ++i) {
        int16_t value = new_position[i];
        uint8_t bytes[2];
        bytes[0] = value & 0xFF;
        bytes[1] = (value >> 8) & 0xFF;
        write(serial_port, bytes, 2);
    }
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
