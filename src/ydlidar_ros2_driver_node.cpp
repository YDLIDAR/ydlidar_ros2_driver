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
#include <thread>
#include <iostream>
#include <memory>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>
#include <string>
#include <signal.h>

#define ROS2Version "1.0.1"


struct LidarParams {
    std::string port;
    std::string ignore_array;
    std::string frame_id;
    int baudrate;
    int lidar_type;
    int device_type;
    int sample_rate;
    int abnormal_check_count;
    int intensity_bit;
    bool fixed_resolution;
    bool reversion;
    bool inverted;
    bool auto_reconnect;
    bool isSingleChannel;
    bool intensity;
    bool support_motor_dtr;
    bool sun_noise_filter;
    bool glass_noise_filter;
    bool invalid_range_is_inf;
    float angle_max;
    float angle_min;
    float range_max;
    float range_min;
    float frequency;
};


class YDLidarNode : public rclcpp::Node
{
public:
YDLidarNode() : Node("ydlidar_ros2_driver_node")
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n",
  ((std::string)ROS2Version).c_str());

  node_name = get_name();
  node_namespace = get_namespace();

  // Parameter declaration and retrieval
  declare_parameters();
  get_parameters();

  param_cb_handle = 
  this->add_on_set_parameters_callback(
    std::bind(&YDLidarNode::on_param_change, this, std::placeholders::_1)
  );
  
  // Set properties on the laser based on parameters
  set_property(lidar_param);
  get_property();
  info();

  // Initialization
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  if (!initialize_laser()) {
    RCLCPP_ERROR(get_logger(), "[YDLIDAR ERROR] Failed to initialize laser");
    rclcpp::shutdown();
    return;
  }

  // Create publishers and services
  create_publishers();
  create_services();
  // Start scan loop in a separate thread
  scan_thread_ = std::thread(&YDLidarNode::scan_loop, this);
  param_worker_ = std::thread(&YDLidarNode::param_worker_loop, this);
  RCLCPP_INFO(get_logger(), "[YDLIDAR INFO] Laser initialized successfully");
};

~YDLidarNode()
{
    running_ = false;

    if (scan_thread_.joinable())
        scan_thread_.join();

    stop_laser();
};

private:
  ///////////////////////////////////////////////////////////////////////////
  // Parameters
  ///////////////////////////////////////////////////////////////////////////
	void declare_parameters()
	{
    this->declare_parameter<std::string>("port", "/dev/ydlidar");
    this->declare_parameter<std::string>("ignore_array", "");
    this->declare_parameter<std::string>("frame_id", "laser_frame");
    this->declare_parameter<int>("baudrate", 230400);
    this->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
    this->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
    this->declare_parameter<int>("sample_rate", 9);
    this->declare_parameter<int>("abnormal_check_count", 4);
    this->declare_parameter<int>("intensity_bit", 8);
    this->declare_parameter<bool>("fixed_resolution", false);
    this->declare_parameter<bool>("reversion", true);
    this->declare_parameter<bool>("inverted", true);
    this->declare_parameter<bool>("auto_reconnect", true);
    this->declare_parameter<bool>("isSingleChannel", false);
    this->declare_parameter<bool>("intensity", false);
    this->declare_parameter<bool>("support_motor_dtr", false);
    this->declare_parameter<bool>("sun_noise_filter", false);
    this->declare_parameter<bool>("glass_noise_filter", false);
    this->declare_parameter<bool>("invalid_range_is_inf", false);
    this->declare_parameter<float>("angle_max", 180.0f);
    this->declare_parameter<float>("angle_min", -180.0f);
    this->declare_parameter<float>("range_max", 64.0f);
    this->declare_parameter<float>("range_min", 0.1f);
    this->declare_parameter<float>("frequency", 10.0f);
	};

	void get_parameters()
	{
    lidar_param.port         = this->get_parameter("port").as_string();
    lidar_param.ignore_array = this->get_parameter("ignore_array").as_string();
    lidar_param.frame_id     = this->get_parameter("frame_id").as_string();
    lidar_param.baudrate     = this->get_parameter("baudrate").as_int();
    lidar_param.lidar_type   = this->get_parameter("lidar_type").as_int();
    lidar_param.device_type  = this->get_parameter("device_type").as_int();
    lidar_param.sample_rate  = this->get_parameter("sample_rate").as_int();
    lidar_param.abnormal_check_count = this->get_parameter("abnormal_check_count").as_int();
    lidar_param.intensity_bit        = this->get_parameter("intensity_bit").as_int();
    lidar_param.fixed_resolution     = this->get_parameter("fixed_resolution").as_bool();
    lidar_param.reversion            = this->get_parameter("reversion").as_bool();
    lidar_param.inverted             = this->get_parameter("inverted").as_bool();
    lidar_param.auto_reconnect       = this->get_parameter("auto_reconnect").as_bool();
    lidar_param.isSingleChannel      = this->get_parameter("isSingleChannel").as_bool();
    lidar_param.intensity            = this->get_parameter("intensity").as_bool();
    lidar_param.support_motor_dtr    = this->get_parameter("support_motor_dtr").as_bool();
    lidar_param.sun_noise_filter     = this->get_parameter("sun_noise_filter").as_bool();
    lidar_param.glass_noise_filter   = this->get_parameter("glass_noise_filter").as_bool();
    lidar_param.invalid_range_is_inf = this->get_parameter("invalid_range_is_inf").as_bool();
    lidar_param.angle_max = this->get_parameter("angle_max").as_double();
    lidar_param.angle_min = this->get_parameter("angle_min").as_double();
    lidar_param.range_max = this->get_parameter("range_max").as_double();
    lidar_param.range_min = this->get_parameter("range_min").as_double();
    lidar_param.frequency = this->get_parameter("frequency").as_double();

	};

	void info()
	{
    RCLCPP_INFO(get_logger(), "port:%s", lidar_param.port.c_str());
    RCLCPP_INFO(get_logger(), "ignore_array:%s", lidar_param.ignore_array.c_str());
    RCLCPP_INFO(get_logger(), "frame_id:%s", lidar_param.frame_id.c_str());
    RCLCPP_INFO(get_logger(), "baudrate:%d", lidar_param.baudrate);
    RCLCPP_INFO(get_logger(), "lidar_type:%d", lidar_param.lidar_type);
    RCLCPP_INFO(get_logger(), "device_type:%d", lidar_param.device_type);
    RCLCPP_INFO(get_logger(), "sample_rate:%d", lidar_param.sample_rate);
    RCLCPP_INFO(get_logger(), "abnormal_check_count:%d", lidar_param.abnormal_check_count);
    RCLCPP_INFO(get_logger(), "intensity_bit:%d", lidar_param.intensity_bit);
    RCLCPP_INFO(get_logger(), "fixed_resolution:%s", (lidar_param.fixed_resolution ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "reversion:%s", (lidar_param.reversion ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "inverted:%s", (lidar_param.inverted ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "auto_reconnect:%s", (lidar_param.auto_reconnect ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "isSingleChannel:%s", (lidar_param.isSingleChannel ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "intensity:%s", (lidar_param.intensity ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "support_motor_dtr:%s", (lidar_param.support_motor_dtr ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "sun_noise_filter:%s", (lidar_param.sun_noise_filter ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "glass_noise_filter:%s", (lidar_param.glass_noise_filter ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "invalid_range_is_inf:%s", (lidar_param.invalid_range_is_inf ? "True" : "False"));
    RCLCPP_INFO(get_logger(), "angle_max:%f", lidar_param.angle_max);
    RCLCPP_INFO(get_logger(), "angle_min:%f", lidar_param.angle_min);
    RCLCPP_INFO(get_logger(), "range_max:%f", lidar_param.range_max);
    RCLCPP_INFO(get_logger(), "range_min:%f", lidar_param.range_min);
    RCLCPP_INFO(get_logger(), "frequency:%f", lidar_param.frequency);

	};

  bool set_property(LidarParams &property)
  {
    if (!laser.setlidaropt(LidarPropSerialPort, property.port.c_str(), property.port.size())) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropIgnoreArray, property.ignore_array.c_str(), property.ignore_array.size())) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropSerialBaudrate, &property.baudrate, sizeof(int))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropLidarType, &property.lidar_type, sizeof(int))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropDeviceType, &property.device_type, sizeof(int))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropSampleRate, &property.sample_rate, sizeof(int))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropAbnormalCheckCount, &property.abnormal_check_count, sizeof(int))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropIntenstiyBit, &property.intensity_bit, sizeof(int))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropFixedResolution, &property.fixed_resolution, sizeof(bool))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropReversion, &property.reversion, sizeof(bool))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropInverted, &property.inverted, sizeof(bool))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropAutoReconnect, &property.auto_reconnect, sizeof(bool))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropSingleChannel, &property.isSingleChannel, sizeof(bool))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropIntenstiy, &property.intensity, sizeof(bool))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &property.support_motor_dtr, sizeof(bool))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropMaxAngle, &property.angle_max, sizeof(float))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropMinAngle, &property.angle_min, sizeof(float))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropMaxRange, &property.range_max, sizeof(float))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropMinRange, &property.range_min, sizeof(float))) {
      return false;
    }
    if (!laser.setlidaropt(LidarPropScanFrequency, &property.frequency, sizeof(float))) {
      return false;
    }
    laser.enableSunNoise(property.sun_noise_filter);
    laser.enableGlassNoise(property.glass_noise_filter);

    return true;
};

  void get_property()
  {
    laser.getlidaropt(LidarPropSerialPort, &lidar_param.port[0], lidar_param.port.size());
    laser.getlidaropt(LidarPropIgnoreArray, &lidar_param.ignore_array[0], lidar_param.ignore_array.size());
    laser.getlidaropt(LidarPropSerialBaudrate, &lidar_param.baudrate, sizeof(int));
    laser.getlidaropt(LidarPropLidarType, &lidar_param.lidar_type, sizeof(int));
    laser.getlidaropt(LidarPropDeviceType, &lidar_param.device_type, sizeof(int));
    laser.getlidaropt(LidarPropSampleRate, &lidar_param.sample_rate, sizeof(int));
    laser.getlidaropt(LidarPropAbnormalCheckCount, &lidar_param.abnormal_check_count, sizeof(int));
    laser.getlidaropt(LidarPropIntenstiyBit, &lidar_param.intensity_bit, sizeof(int));
    laser.getlidaropt(LidarPropFixedResolution, &lidar_param.fixed_resolution, sizeof(bool));
    laser.getlidaropt(LidarPropReversion, &lidar_param.reversion, sizeof(bool));
    laser.getlidaropt(LidarPropInverted, &lidar_param.inverted, sizeof(bool));
    laser.getlidaropt(LidarPropAutoReconnect, &lidar_param.auto_reconnect, sizeof(bool));
    laser.getlidaropt(LidarPropSingleChannel, &lidar_param.isSingleChannel, sizeof(bool));
    laser.getlidaropt(LidarPropIntenstiy, &lidar_param.intensity, sizeof(bool));
    laser.getlidaropt(LidarPropSupportMotorDtrCtrl, &lidar_param.support_motor_dtr, sizeof(bool));
    laser.getlidaropt(LidarPropMaxAngle, &lidar_param.angle_max, sizeof(float));
    laser.getlidaropt(LidarPropMinAngle, &lidar_param.angle_min, sizeof(float));
    laser.getlidaropt(LidarPropMaxRange, &lidar_param.range_max, sizeof(float));
    laser.getlidaropt(LidarPropMinRange, &lidar_param.range_min, sizeof(float));
    laser.getlidaropt(LidarPropScanFrequency, &lidar_param.frequency, sizeof(float));
  };

rcl_interfaces::msg::SetParametersResult 
on_param_change(const std::vector<rclcpp::Parameter> &params) 
{
    std::lock_guard<std::mutex> lock(param_mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    LidarParams temp_params = lidar_param;
    bool needs_reset = true, updated = true;
    
    // Parameters that DON'T require reconnection
    std::vector<std::string> safe_params = {
    "frequency", "angle_max", "angle_min", 
    "range_max", "range_min",
    "intensity", "intensity_bit",
    "fixed_resolution", "reversion", "inverted",
    "ignore_array", "sun_noise_filter", "glass_noise_filter"
    };

    for (const auto &param : params) {
        const auto& name = param.get_name();
        
        if(std::find(std::begin(safe_params), std::end(safe_params), name) != std::end(safe_params)){
          needs_reset = false;
          RCLCPP_INFO(this->get_logger(),"[YDLIDAR] Parameter does not require laser reset");
        }

        if (name == "frequency") {
            temp_params.frequency = param.as_double();
        } else if (name == "angle_max") {
            temp_params.angle_max = param.as_double();
        } else if (name == "angle_min") {
            temp_params.angle_min = param.as_double();
        } else if (param.get_name() == "port") {
            temp_params.port = param.as_string();
        } else if (param.get_name() == "ignore_array") {
            temp_params.ignore_array = param.as_string();
        } else if (param.get_name() == "baudrate") {
            temp_params.baudrate = param.as_int();
        } else if (param.get_name() == "lidar_type") {
            temp_params.lidar_type = param.as_int();
        } else if (param.get_name() == "device_type") {
            temp_params.device_type = param.as_int();
        } else if (param.get_name() == "sample_rate") {
            temp_params.sample_rate = param.as_int();
        } else if (param.get_name() == "abnormal_check_count") {
            temp_params.abnormal_check_count = param.as_int();
        } else if (param.get_name() == "intensity_bit") {
            temp_params.intensity_bit = param.as_int();
        } else if (param.get_name() == "fixed_resolution") {
            temp_params.fixed_resolution = param.as_bool();
        } else if (param.get_name() == "reversion") {
            temp_params.reversion = param.as_bool();
        } else if (param.get_name() == "inverted") {
            temp_params.inverted = param.as_bool();
        } else if (param.get_name() == "auto_reconnect") {
            temp_params.auto_reconnect = param.as_bool();
        } else if (param.get_name() == "isSingleChannel") {
            temp_params.isSingleChannel = param.as_bool();
        } else if (param.get_name() == "intensity") {
            temp_params.intensity = param.as_bool();
        } else if (param.get_name() == "support_motor_dtr") {
            temp_params.support_motor_dtr = param.as_bool();
        } else if (param.get_name() == "sun_noise_filter") {
            temp_params.sun_noise_filter = param.as_bool();
        } else if (param.get_name() == "glass_noise_filter") {
            temp_params.glass_noise_filter = param.as_bool();
        } else if (param.get_name() == "invalid_range_is_inf") {
            temp_params.invalid_range_is_inf = param.as_bool();
        } else if (param.get_name() == "range_max") {
            temp_params.range_max = param.as_double();
        } else if (param.get_name() == "range_min") {
            temp_params.range_min = param.as_double();
        } else{
            result.successful = false;
            updated = false;
        }
    }
    if (updated){
      // 1. Try to apply new properties
      if (!set_property(temp_params)) {
          result.successful = false;
          return result;  // Bail out, don't touch lidar_param
      }
      // 2. Pause scan loop
      if (needs_reset){
        RCLCPP_INFO(this->get_logger(),"[YDLIDAR] Needs reset after parameter change");
        running_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 3. Restart laser to make properties take effect
        if (!reset_laser()) {
            result.successful = false;
            running_ = true;
            return result;  // Failed to restart, lidar_param stays old
        }

        lidar_param = temp_params;
        running_ = true;
        result.successful = true;
      }
    }
    return result;
  }


  void create_publishers()
  {
    laser_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "~/scan", rclcpp::SensorDataQoS());
    pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "~/point_cloud", rclcpp::SensorDataQoS());
    };

void create_services()
{
  auto stop_scan_service =
    [this](const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response> ) -> bool
  {
    return stop_laser();
    };

  stop_service = this->create_service<std_srvs::srv::Empty>(
      "~/stop_scan", stop_scan_service);

  auto start_scan_service =
    [this](const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool
  {
    return start_laser();
    };

  start_service = this->create_service<std_srvs::srv::Empty>(
      "~/start_scan", start_scan_service);
};


  ///////////////////////////////////////////////////////////////////////////
  // Initialization
  // The T-mini Pro powers on in idle mode (per spec section 1, Development
  // Manual). The SDK connects and immediately queries health [A5 92] and
  // device info [A5 90]. If the device hasn't finished its boot sequence the
  // health query times out with 0xffffffff, leaving the driver in a state
  // where startScan() also fails. A short delay here gives the device time
  // to finish booting before the first serial command is sent.
  ///////////////////////////////////////////////////////////////////////////

  bool initialize_laser()
  {
    bool ret = laser.initialize();
    for (int i = 0; i < 3 && !ret; i++) {
      RCLCPP_ERROR(this->get_logger(),
                  "[YDLIDAR] Failed to initialize: %s. Retrying... (%d/3)",
                  laser.DescribeError(), i + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      ret = laser.initialize();
    }
    if(!ret) {
      RCLCPP_ERROR(this->get_logger(),
                  "[YDLIDAR] Failed to initialize after 3 attempts: %s",
                  laser.DescribeError());
      rclcpp::shutdown();
      return false;
    }

    ret = laser.turnOn();
    for (int i = 0; i < 3 && !ret; i++) {
      // If turnOn() fails, the driver may be in a bad state. Try power cycling the device before retrying.
      stop_laser();
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));

      ret = laser.initialize();
      if (!ret) {
        RCLCPP_ERROR(this->get_logger(),
                    "[YDLIDAR] Failed to initialize: %s. Retrying... (%d/3)",
                    laser.DescribeError(), i + 1);
        continue;
      }
      ret = laser.turnOn();
      if (!ret) {
        RCLCPP_ERROR(this->get_logger(),
                    "[YDLIDAR] Failed to start scan: %s (driver error: %d). Retrying... (%d/3)",
                    laser.DescribeError(),
                    static_cast<int>(laser.getDriverError()), i + 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      }
    }
    return ret;
  };

  bool stop_laser()
  {
    if (!laser.turnOff()) {
      return false;
    }
    laser.disconnecting();
    return true;
  };

  bool start_laser()
  {
    if (!laser.initialize()) {
      return false;
    }
    if (!laser.turnOn()) {
      return false;
    }
    return true;
  };

  bool reset_laser()
  {
    if (!stop_laser()) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    if (!start_laser()) {
      return false;
    }
    return true;
  };


  ///////////////////////////////////////////////////////////////////////////
  // Scan loop
  ///////////////////////////////////////////////////////////////////////////
  void scan_loop()
  { 
    while (running_ && rclcpp::ok()) {
      LaserScan scan;

      if (laser.doProcessSimple(scan)) {

        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        auto pc_msg   = std::make_shared<sensor_msgs::msg::PointCloud>();

        scan_msg->header.stamp.sec     = RCL_NS_TO_S(scan.stamp);
        scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
        scan_msg->header.frame_id      = lidar_param.frame_id;
        pc_msg->header                 = scan_msg->header;

        scan_msg->angle_min       = scan.config.min_angle;
        scan_msg->angle_max       = scan.config.max_angle;
        scan_msg->angle_increment = scan.config.angle_increment;
        scan_msg->scan_time       = scan.config.scan_time;
        scan_msg->time_increment  = scan.config.time_increment;
        scan_msg->range_min       = scan.config.min_range;
        scan_msg->range_max       = scan.config.max_range;

        int size = static_cast<int>(
            (scan.config.max_angle - scan.config.min_angle) /
            scan.config.angle_increment) + 1;
        scan_msg->ranges.resize(size);
        scan_msg->intensities.resize(size);

        pc_msg->channels.resize(2);
        int idx_intensity = 0;
        pc_msg->channels[idx_intensity].name = "intensities";
        int idx_timestamp = 1;
        pc_msg->channels[idx_timestamp].name = "stamps";

        for (size_t i = 0; i < scan.points.size(); i++) {
          int index = static_cast<int>(std::ceil(
              (scan.points[i].angle - scan.config.min_angle) /
              scan.config.angle_increment));

          if (index >= 0 && index < size) {
            if (scan.points[i].range >= scan.config.min_range) {
              scan_msg->ranges[index]      = scan.points[i].range;
              scan_msg->intensities[index] = scan.points[i].intensity;
            }
          }

          if (scan.points[i].range >= scan.config.min_range &&
              scan.points[i].range <= scan.config.max_range) {
            geometry_msgs::msg::Point32 point;
            point.x = scan.points[i].range * cos(scan.points[i].angle);
            point.y = scan.points[i].range * sin(scan.points[i].angle);
            point.z = 0.0;
            pc_msg->points.push_back(point);
            pc_msg->channels[idx_intensity].values.push_back(scan.points[i].intensity);
            pc_msg->channels[idx_timestamp].values.push_back(
                static_cast<float>(i) * scan.config.time_increment);
          }
        }

        laser_pub->publish(*scan_msg);
        pc_pub->publish(*pc_msg);

      } else {
        RCLCPP_WARN(this->get_logger(),
                    "[YDLIDAR] Failed to get scan (driver error: %d, scanning: %s)",
                    static_cast<int>(laser.getDriverError()),
                    laser.isScanning() ? "yes" : "no");
      }

      if (!rclcpp::ok()) {
        stop_laser();
        break;
      }

    }
  };

  void param_worker_loop() {
      while (rclcpp::ok()) {
          std::unique_lock<std::mutex> lock(param_work_mutex_);
          param_work_cv_.wait(lock, [this] { return param_work_pending_ || !running_; });
          
          if (!running_) break;
          if (param_work_pending_) {
              set_property(pending_params_) && reset_laser();
              lidar_param = pending_params_;
              param_work_pending_ = false;
          }
      }
  }
  
  ///////////////////////////////////////////////////////////////////////////
  // Variables
  ///////////////////////////////////////////////////////////////////////////
  std::string node_name, node_namespace;

  std::thread scan_thread_, param_worker_;
  std::atomic<bool> running_{true};

  CYdLidar laser;

  LidarParams lidar_param;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pc_pub;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service;
  std::mutex param_mutex_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle;

  std::mutex param_work_mutex_;
  std::condition_variable param_work_cv_;
  bool param_work_pending_ = false;
  LidarParams pending_params_;

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  auto node = std::make_shared<YDLidarNode>();
  executor.add_node(node);
  executor.spin();
  
  executor.remove_node(node);
  rclcpp::shutdown();
  
  return 0;
}