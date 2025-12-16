/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(),
              "[YDLIDAR INFO] Current ROS Driver Version: %s\n",
              ((std::string)ROS2Verision).c_str());

  CYdLidar laser;

  std::string str_optvalue = "/dev/ydlidar";
  node->declare_parameter<std::string>("port", "/dev/ydlidar");
  node->get_parameter("port", str_optvalue);
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  str_optvalue = "";
  node->declare_parameter<std::string>("ignore_array", "");
  node->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = "laser_link";
  node->declare_parameter<std::string>("frame_id", "laser_link");
  node->get_parameter("frame_id", frame_id);

  ////////// int parameters //////////
  int optval = 230400;
  node->declare_parameter<int>("baudrate", 230400);
  node->get_parameter("baudrate", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));

  optval = TYPE_TRIANGLE;
  node->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
  node->get_parameter("lidar_type", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

  optval = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
  node->get_parameter("device_type", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

  optval = 9;
  node->declare_parameter<int>("sample_rate", 9);
  node->get_parameter("sample_rate", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

  optval = 4;
  node->declare_parameter<int>("abnormal_check_count", 4);
  node->get_parameter("abnormal_check_count", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  optval = 0;
  node->declare_parameter<int>("intensity_bit", 0);
  node->get_parameter("intensity_bit", optval);
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  ////////// bool parameters //////////
  bool b_optvalue = false;
  node->declare_parameter<bool>("fixed_resolution", false);
  node->get_parameter("fixed_resolution", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));

  b_optvalue = true;
  node->declare_parameter<bool>("reversion", true);
  node->get_parameter("reversion", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));

  b_optvalue = true;
  node->declare_parameter<bool>("inverted", true);
  node->get_parameter("inverted", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));

  b_optvalue = true;
  node->declare_parameter<bool>("auto_reconnect", true);
  node->get_parameter("auto_reconnect", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

  b_optvalue = false;
  node->declare_parameter<bool>("isSingleChannel", false);
  node->get_parameter("isSingleChannel", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));

  b_optvalue = false;
  node->declare_parameter<bool>("intensity", false);
  node->get_parameter("intensity", b_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));

  b_optvalue = false;
  node->declare_parameter<bool>("support_motor_dtr", false);
  node->get_parameter("support_motor_dtr", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  b_optvalue = false;
  node->declare_parameter<bool>("debug", false);
  node->get_parameter("debug", b_optvalue);
  laser.setEnableDebug(b_optvalue);

  ////////// float parameters //////////
  float f_optvalue = 180.0f;
  node->declare_parameter<float>("angle_max", 180.0f);
  node->get_parameter("angle_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));

  f_optvalue = -180.0f;
  node->declare_parameter<float>("angle_min", -180.0f);
  node->get_parameter("angle_min", f_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  f_optvalue = 64.0f;
  node->declare_parameter<float>("range_max", 64.0f);
  node->get_parameter("range_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));

  f_optvalue = 0.1f;
  node->declare_parameter<float>("range_min", 0.1f);
  node->get_parameter("range_min", f_optvalue);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

  f_optvalue = 10.0f;
  node->declare_parameter<float>("frequency", 10.0f);
  node->get_parameter("frequency", f_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter<bool>("invalid_range_is_inf", false);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  // initialize
  bool ret = laser.initialize();
  if (ret)
  {
    int i_v = 0;
    node->declare_parameter<int>("m1_mode", 0);
    node->get_parameter("m1_mode", i_v);
    laser.setWorkMode(i_v, 0x01);

    i_v = 0;
    node->declare_parameter<int>("m2_mode", 0);
    node->get_parameter("m2_mode", i_v);
    laser.setWorkMode(i_v, 0x02);

    i_v = 1;
    node->declare_parameter<int>("m3_mode", 1);
    node->get_parameter("m3_mode", i_v);
    laser.setWorkMode(i_v, 0x04);

    ret = laser.turnOn();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }

  auto laser_pub =
      node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  auto stop_scan_service =
      [&laser](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool {
        return laser.turnOff();
      };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

  auto start_scan_service =
      [&laser](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool {
        return laser.turnOn();
      };

  auto start_service =
      node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok())
  {
    LaserScan scan;
    if (laser.doProcessSimple(scan))
    {
      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp = node->now();
      scan_msg->header.frame_id = frame_id;

      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;

      int size =
          (scan.config.max_angle - scan.config.min_angle) /
              scan.config.angle_increment +
          1;

      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);

      for (size_t i = 0; i < scan.points.size(); i++)
      {
        const auto &p = scan.points.at(i);
        int index = std::ceil((p.angle - scan.config.min_angle) /
                              scan.config.angle_increment);
        if (index >= 0 && index < size)
        {
          scan_msg->ranges[index] = p.range;
          scan_msg->intensities[index] = p.intensity;
        }
      }
      laser_pub->publish(*scan_msg);
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }

    if (!rclcpp::ok())
    {
      break;
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(),
              "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
