/*
 *  Two YDLIDARs Mixer
 *  Enjoy Robotics
 */

#include "rclcpp/rclcpp.hpp"
#include "src/CYdLidar.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)

sensor_msgs::msg::LaserScan scan_msg;
sensor_msgs::msg::PointCloud2 cloud_msg;

std::vector<uint8_t> cloud1;
std::vector<uint8_t> cloud2;
std::vector<uint8_t> cloud;

uint32_t width1;
uint32_t width2;

int size;

void pointcloudCb1 (const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs)
{
  cloud1 = point_cloud2_msgs->data;
  width1 = point_cloud2_msgs->width;

  // printf("[YDLIDAR INFO1]: data_size-point_step : [%ld, %d]\n",  cloud1.size(), cloud1[0]);
}

void pointcloudCb2 (const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs)
{
  cloud2 = point_cloud2_msgs->data;
  width2 = point_cloud2_msgs->width;

  cloud_msg.header.stamp.sec = point_cloud2_msgs -> header.stamp.sec;
  cloud_msg.header.stamp.nanosec = point_cloud2_msgs->header.stamp.nanosec;
  cloud_msg.header.frame_id = point_cloud2_msgs->header.frame_id;
  cloud_msg.fields = point_cloud2_msgs->fields;
  cloud_msg.height = point_cloud2_msgs->height;
  cloud_msg.is_bigendian = point_cloud2_msgs->is_bigendian;
  cloud_msg.width = width1 + width2;
  cloud_msg.point_step = point_cloud2_msgs -> point_step;
  cloud_msg.row_step = point_cloud2_msgs->row_step;
  cloud_msg.is_dense = point_cloud2_msgs->is_dense;

  // printf("[YDLIDAR INFO2]: data_size-point_step : [%ld, %d]\n",  cloud2.size(), cloud2[0]);

  cloud.erase(cloud.begin(),cloud.end());

  cloud.reserve(cloud.size() + cloud1.size());
  cloud.insert(cloud.end(), cloud1.begin(), cloud1.end());
  cloud.reserve(cloud.size() + cloud2.size());
  cloud.insert(cloud.end(), cloud2.begin(), cloud2.end());

  // printf("[YDLIDAR INFO]: data_size-point_step : [%ld, %d]\n",  cloud.size(), cloud[0]);
  cloud_msg.data = cloud;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_client");

  auto lidar_info_sub1 = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                        "/scanner/cloud1", rclcpp::SensorDataQoS(), pointcloudCb1);

  auto lidar_info_sub2 = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                        "/scanner/cloud2", rclcpp::SensorDataQoS(), pointcloudCb2);

  // auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  auto pointcloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());

  rclcpp::WallRate loop_rate(20);

  while (rclcpp::ok()) {

    pointcloud_pub->publish(cloud_msg);

    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
