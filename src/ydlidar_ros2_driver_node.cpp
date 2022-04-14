/*
 *    YDLIDAR SYSTEM
 *    YDLIDAR ROS 2 Node
 *
 *    Copyright 2017 - 2020 EAI TEAM
 *    http://www.eaibot.com
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

using namespace std::chrono_literals;

class YDLidarClass: public rclcpp::Node
{
    public:
        YDLidarClass(): Node("ydlidar_ros2_driver_node")
        {
            scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

            std::string str_optvalue;
            this->declare_parameter<std::string>("port", "/dev/ydlidar");
            this->get_parameter("port", str_optvalue);
            ///lidar port
            laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

            ///ignore array
            this->declare_parameter<std::string>("ignore_array", "");
            this->get_parameter("ignore_array", str_optvalue);
            laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

            this->declare_parameter<std::string>("frame_id", "laser_frame");
            this->get_parameter("frame_id", frame_id);

            this->declare_parameter<std::string>("topic", "scan");
            this->get_parameter("topic", topic);


            //////////////////////int property/////////////////
            int optval;
            /// lidar baudrate
            this->declare_parameter<int>("baudrate", 230400);
            this->get_parameter("baudrate", optval);
            laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
            /// tof lidar
            this->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
            this->get_parameter("lidar_type", optval);
            laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
            /// device type
            this->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
            this->get_parameter("device_type", optval);
            laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
            /// sample rate
            this->declare_parameter<int>("sample_rate", 9);
            this->get_parameter("sample_rate", optval);
            laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
            /// abnormal count
            this->declare_parameter<int>("abnormal_check_count", 4);
            this->get_parameter("abnormal_check_count", optval);
            laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));


            //////////////////////bool property/////////////////
            bool b_optvalue;
            /// fixed angle resolution
            this->declare_parameter<bool>("fixed_resolution", false);
            this->get_parameter("fixed_resolution", b_optvalue);
            laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
            /// rotate 180
            this->declare_parameter<bool>("reversion", true);
            this->get_parameter("reversion", b_optvalue);
            laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
            /// Counterclockwise
            this->declare_parameter<bool>("inverted", true);
            this->get_parameter("inverted", b_optvalue);
            laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));

            this->declare_parameter<bool>("auto_reconnect", true);
            this->get_parameter("auto_reconnect", b_optvalue);
            laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
            /// one-way communication
            this->declare_parameter<bool>("isSingleChannel", false);
            this->get_parameter("isSingleChannel", b_optvalue);
            laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
            /// intensity
            this->declare_parameter<bool>("intensity", false);
            this->get_parameter("intensity", b_optvalue);
            laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
            /// Motor DTR
            this->declare_parameter<bool>("support_motor_dtr", false);
            this->get_parameter("support_motor_dtr", b_optvalue);
            laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));


            //////////////////////float property/////////////////
            float f_optvalue;
            /// unit: °
            this->declare_parameter<float>("angle_max", 180.f);
            this->get_parameter("angle_max", f_optvalue);
            laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));

            this->declare_parameter<float>("angle_min", -180.f);
            this->get_parameter("angle_min", f_optvalue);
            laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
            /// unit: m
            this->declare_parameter<float>("range_max", 64.f);
            this->get_parameter("range_max", f_optvalue);
            laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));

            this->declare_parameter<float>("range_min", 0.1f);
            this->get_parameter("range_min", f_optvalue);
            laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
            /// unit: Hz
            this->declare_parameter<float>("frequency", 10.f);
            this->get_parameter("frequency", f_optvalue);
            laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));


            laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS());

            bool ret = laser.initialize();
            if (ret) {
                ret = laser.turnOn();
            } else {
                RCLCPP_ERROR(this->get_logger(), "%s\n", laser.DescribeError());
            }

            timer_ = this->create_wall_timer(50ms, std::bind(&YDLidarClass::tick, this));
        }


        void tick()
        {
            if (laser.doProcessSimple(scan)) {

                scan_msg->header.frame_id = frame_id;
                scan_msg->header.stamp = get_clock()->now();
                scan_msg->angle_min = scan.config.min_angle;
                scan_msg->angle_max = scan.config.max_angle;
                scan_msg->angle_increment = scan.config.angle_increment;
                scan_msg->scan_time = scan.config.scan_time;
                scan_msg->time_increment = scan.config.time_increment;
                scan_msg->range_min = scan.config.min_range;
                scan_msg->range_max = scan.config.max_range;

                int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
                scan_msg->ranges.resize(size);
                scan_msg->intensities.resize(size);
                for(size_t i=0; i < scan.points.size(); i++) {
                    int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
                    if(index >=0 && index < size) {
                      scan_msg->ranges[index] = scan.points[i].range;
                      scan_msg->intensities[index] = scan.points[i].intensity;
                    }
                }

                laser_publisher_->publish(*scan_msg);

            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get scan");
            }
        }


        ~YDLidarClass()
        {
            RCLCPP_INFO(this->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
            laser.turnOff();
            laser.disconnecting();
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
        CYdLidar laser;
        LaserScan scan;
        sensor_msgs::msg::LaserScan::SharedPtr scan_msg;
        std::string frame_id, topic;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<YDLidarClass>());
    rclcpp::shutdown();
    return 0;
}
