#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "lsm9ds1_driver.hpp" // Include the driver header file

class LSM9DS1Interface : public rclcpp::Node
{
public:
  LSM9DS1Interface() : Node("lsm9ds1_interface_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

    // Initialize LSM9DS1 driver
    lsm9ds1_driver_ = std::make_unique<LSM9DS1Driver>("/dev/i2c-0");
    

    // Set up publishing loop
    auto publish_callback = [this]() -> void {
      auto imu_data = lsm9ds1_driver_->readImuData();

      auto imu_msg = sensor_msgs::msg::Imu();
      imu_msg.header.stamp = this->now();
      imu_msg.header.frame_id = "imu_link";

      imu_msg.linear_acceleration.x = imu_data.accelerometer_x;
      imu_msg.linear_acceleration.y = imu_data.accelerometer_y;
      imu_msg.linear_acceleration.z = imu_data.accelerometer_z;
      imu_msg.angular_velocity.x = imu_data.gyroscope_x;
      imu_msg.angular_velocity.y = imu_data.gyroscope_y;
      imu_msg.angular_velocity.z = imu_data.gyroscope_z;

      publisher_->publish(imu_msg);
    };

    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), publish_callback);
  }

private:
  std::unique_ptr<LSM9DS1Driver> lsm9ds1_driver_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LSM9DS1Interface>());
  rclcpp::shutdown();
  return 0;
}
