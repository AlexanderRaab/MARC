#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>
#include <functional>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <camsense_x1_ros2/CamsenseX1.h>

using namespace std::chrono_literals;

#define TOPIC "scan"
#define DEVICE "/dev/ttyUSB0"
#define FRAME "lidar_link"
#define QOS_PROFIL rclcpp::QoS(10).best_effort().keep_last(5)

class CamsensePublisher : public rclcpp::Node
{
  public:
    CamsensePublisher(): Node("camsense_x1")
    {
      // Init parameters
      this->declare_parameter("frame", FRAME);
      this->declare_parameter("dev", DEVICE);
      this->declare_parameter("topic", TOPIC);

      // Init publisher and serial port
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(this->get_parameter("topic").as_string(), QOS_PROFIL);
      this->lidar = new Camsense::CamsenseX1(this->get_parameter("dev").as_string());

      // Populate static message fields
      init_msg();

      RCLCPP_INFO(this->get_logger(), "Initialization done.");
    }

    /* Reads the next scan and publishes the meassurement data
    
    */
    void read_and_publish()
    {
      // Read scan
      lidar->readScan();

      // Fill header and clear message data
      msg_.header.stamp = this->get_clock()->now();
      msg_.ranges.clear();
      msg_.intensities.clear();

      // Fill measurement data
      msg_.time_increment = lidar->getTimeIncrement();
      msg_.scan_time = 1/lidar->getSpeed();
      msg_.ranges.insert(msg_.ranges.begin(), &lidar->distances[0], &lidar->distances[0]+lidar->DATA_LEN);
      msg_.intensities.insert(msg_.intensities.begin(), &lidar->qualities[0], &lidar->qualities[0]+lidar->DATA_LEN);

      // Publish
      publisher_->publish(msg_);
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    Camsense::CamsenseX1 *lidar;
    sensor_msgs::msg::LaserScan msg_ = sensor_msgs::msg::LaserScan();

    /* Populates static message fields
    
    */
    void init_msg()
    {
      msg_.header.frame_id = this->get_parameter("frame").as_string();
      msg_.angle_min = 0.;
      msg_.angle_max = 2*M_PI;
      msg_.angle_increment = -1/lidar->ANGLE_INCREMENT*M_PI/180.;
      msg_.range_min = lidar->MIN_RANGE;
      msg_.range_max = lidar->MAX_RANGE;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CamsensePublisher>();
  RCLCPP_INFO(node->get_logger(), "Spinning...");
  while (rclcpp::ok()) 
  {
    node->read_and_publish();
  }

  RCLCPP_INFO(node->get_logger(), "Stopping...");
  rclcpp::shutdown();
  return 0;
}