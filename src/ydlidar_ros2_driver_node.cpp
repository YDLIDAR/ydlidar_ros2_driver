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
#include <limits>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>

#define ROS2_DRIVER_VERSION "1.0.3"

static constexpr int INIT_RETRIES   = 3;
static constexpr int ERRORS_BEFORE_REBOOT  = 4;
static constexpr int BOOT_DELAY_MS  = 1500;
static constexpr int RESET_DELAY_MS = 1500;


struct LidarParams {
    std::string port              = "/dev/ydlidar";
    std::string ignore_array      = "";
    std::string frame_id          = "laser_frame";
    int baudrate                  = 230400;
    int lidar_type                = TYPE_TRIANGLE;
    int device_type               = YDLIDAR_TYPE_SERIAL;
    int sample_rate               = 9;
    int abnormal_check_count      = 4;
    int intensity_bit             = 8;
    bool fixed_resolution         = false;
    bool reversion                = true;
    bool inverted                 = true;
    bool auto_reconnect           = true;
    bool isSingleChannel          = false;
    bool intensity                = false;
    bool support_motor_dtr        = false;
    bool sun_noise_filter         = false;
    bool glass_noise_filter       = false;
    bool invalid_range_is_inf     = false;
    float angle_max               = 180.0f;
    float angle_min               = -180.0f;
    float range_max               = 64.0f;
    float range_min               = 0.1f;
    float frequency               = 10.0f;
};


class YDLidarNode : public rclcpp::Node
{
public:
YDLidarNode() : Node("ydlidar_ros2_driver_node")
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR] Driver version: %s", ROS2_DRIVER_VERSION);
  node_name = get_name();
  node_namespace = get_namespace();

  declare_parameters();
  lidar_param_ = get_parameters();
  log_parameters(lidar_param_);

  if (!apply_properties(lidar_param_)) {
    throw std::runtime_error("Failed to apply laser properties");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(BOOT_DELAY_MS));

  if (!connect_and_start_laser()) {
    throw std::runtime_error("Failed to Initialize laser!");
  }

  sync_lidar_properties(lidar_param_);
  create_publishers();
  create_services();

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&YDLidarNode::on_param_change, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "[YDLIDAR] Initialized successfully");
}

~YDLidarNode()
{
  // Signal the scan thread to stop
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    running_ = false;
  }
  scan_cv_.notify_all();
 
  // Wait for scan thread to exit with timeout (don't block forever on stuck I/O)
  if (scan_thread_.joinable()) {
    auto timeout = std::chrono::milliseconds(2000);  // 2 second timeout
    auto start = std::chrono::steady_clock::now();
    
    // Busy-wait with small sleep intervals to allow the thread to exit
    while (scan_thread_.joinable()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (std::chrono::steady_clock::now() - start > timeout) {
        RCLCPP_ERROR(get_logger(), "[YDLIDAR] Scan thread did not exit within timeout (likely stuck in SDK call)");
        break;  // Give up gracefully, don't deadlock
      }
    }
    
    // Only join if thread is still joinable
    if (scan_thread_.joinable()) {
      try {
        scan_thread_.join();
      } catch (const std::system_error &e) {
        RCLCPP_ERROR(get_logger(), "[YDLIDAR] system_error joining scan thread: %s", e.what());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "[YDLIDAR] Exception joining scan thread: %s", e.what());
      }
    }
  }
  
  try {
    laser_.turnOff();
    laser_.disconnecting();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "[YDLIDAR] Exception during laser shutdown: %s", e.what());
  }
}

private:
  ///////////////////////////////////////////////////////////////////////////
  // Parameters
  ///////////////////////////////////////////////////////////////////////////

	void declare_parameters()
	{
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Declare ROS Parameters");
    this->declare_parameter<std::string>("port",                 "/dev/ydlidar");
    this->declare_parameter<std::string>("ignore_array",         "");
    this->declare_parameter<std::string>("frame_id",             "laser_frame");
    this->declare_parameter<int>        ("baudrate",             230400);
    this->declare_parameter<int>        ("lidar_type",           TYPE_TRIANGLE);
    this->declare_parameter<int>        ("device_type",          YDLIDAR_TYPE_SERIAL);
    this->declare_parameter<int>        ("sample_rate",          9);
    this->declare_parameter<int>        ("abnormal_check_count", 4);
    this->declare_parameter<int>        ("intensity_bit",        8);
    this->declare_parameter<bool>       ("fixed_resolution",     false);
    this->declare_parameter<bool>       ("reversion",            true);
    this->declare_parameter<bool>       ("inverted",             true);
    this->declare_parameter<bool>       ("auto_reconnect",       true);
    this->declare_parameter<bool>       ("isSingleChannel",      false);
    this->declare_parameter<bool>       ("intensity",            false);
    this->declare_parameter<bool>       ("support_motor_dtr",    false);
    this->declare_parameter<bool>       ("sun_noise_filter",     false);
    this->declare_parameter<bool>       ("glass_noise_filter",   false);
    this->declare_parameter<bool>       ("invalid_range_is_inf", false);
    this->declare_parameter<float>      ("angle_max",            180.0f);
    this->declare_parameter<float>      ("angle_min",           -180.0f);
    this->declare_parameter<float>      ("range_max",            64.0f);
    this->declare_parameter<float>      ("range_min",            0.1f);
    this->declare_parameter<float>      ("frequency",            10.0f);
    }

    LidarParams get_parameters()
	{
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Get ROS Parameters");
    LidarParams p;
    p.port                = this->get_parameter("port").as_string();
    p.ignore_array        = this->get_parameter("ignore_array").as_string();
    p.frame_id            = this->get_parameter("frame_id").as_string();
    p.baudrate            = this->get_parameter("baudrate").as_int();
    p.lidar_type          = this->get_parameter("lidar_type").as_int();
    p.device_type         = this->get_parameter("device_type").as_int();
    p.sample_rate         = this->get_parameter("sample_rate").as_int();
    p.abnormal_check_count= this->get_parameter("abnormal_check_count").as_int();
    p.intensity_bit       = this->get_parameter("intensity_bit").as_int();
    p.fixed_resolution    = this->get_parameter("fixed_resolution").as_bool();
    p.reversion           = this->get_parameter("reversion").as_bool();
    p.inverted            = this->get_parameter("inverted").as_bool();
    p.auto_reconnect      = this->get_parameter("auto_reconnect").as_bool();
    p.isSingleChannel     = this->get_parameter("isSingleChannel").as_bool();
    p.intensity           = this->get_parameter("intensity").as_bool();
    p.support_motor_dtr   = this->get_parameter("support_motor_dtr").as_bool();
    p.sun_noise_filter    = this->get_parameter("sun_noise_filter").as_bool();
    p.glass_noise_filter  = this->get_parameter("glass_noise_filter").as_bool();
    p.invalid_range_is_inf= this->get_parameter("invalid_range_is_inf").as_bool();
    p.angle_max           = static_cast<float>(this->get_parameter("angle_max").as_double());
    p.angle_min           = static_cast<float>(this->get_parameter("angle_min").as_double());
    p.range_max           = static_cast<float>(this->get_parameter("range_max").as_double());
    p.range_min           = static_cast<float>(this->get_parameter("range_min").as_double());
    p.frequency           = static_cast<float>(this->get_parameter("frequency").as_double());
    return p;
    }

    void log_parameters(const LidarParams &p)
	{
    RCLCPP_INFO(get_logger(), "[YDLIDAR] ROS Parameters:");
        RCLCPP_INFO(get_logger(), " - port:                 %s",  p.port.c_str());
        RCLCPP_INFO(get_logger(), " - ignore_array:         %s",  p.ignore_array.c_str());
        RCLCPP_INFO(get_logger(), " - frame_id:             %s",  p.frame_id.c_str());
        RCLCPP_INFO(get_logger(), " - baudrate:             %d",  p.baudrate);
        RCLCPP_INFO(get_logger(), " - lidar_type:           %d",  p.lidar_type);
        RCLCPP_INFO(get_logger(), " - device_type:          %d",  p.device_type);
        RCLCPP_INFO(get_logger(), " - sample_rate:          %d",  p.sample_rate);
        RCLCPP_INFO(get_logger(), " - abnormal_check_count: %d",  p.abnormal_check_count);
        RCLCPP_INFO(get_logger(), " - intensity_bit:        %d",  p.intensity_bit);
        RCLCPP_INFO(get_logger(), " - fixed_resolution:     %s",  p.fixed_resolution     ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - reversion:            %s",  p.reversion            ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - inverted:             %s",  p.inverted             ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - auto_reconnect:       %s",  p.auto_reconnect       ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - isSingleChannel:      %s",  p.isSingleChannel      ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - intensity:            %s",  p.intensity            ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - support_motor_dtr:    %s",  p.support_motor_dtr    ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - sun_noise_filter:     %s",  p.sun_noise_filter     ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - glass_noise_filter:   %s",  p.glass_noise_filter   ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - invalid_range_is_inf: %s",  p.invalid_range_is_inf ? "true" : "false");
        RCLCPP_INFO(get_logger(), " - angle_max:            %.2f", p.angle_max);
        RCLCPP_INFO(get_logger(), " - angle_min:            %.2f", p.angle_min);
        RCLCPP_INFO(get_logger(), " - range_max:            %.2f", p.range_max);
        RCLCPP_INFO(get_logger(), " - range_min:            %.2f", p.range_min);
        RCLCPP_INFO(get_logger(), " - frequency:            %.2f", p.frequency);
    }

    bool apply_properties(const LidarParams &p)
  {
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Apply ROS Parameters to Lidar");
    if (!laser_.setlidaropt(LidarPropSerialPort,          p.port.c_str(),          p.port.size()))          return false;
    if (!laser_.setlidaropt(LidarPropIgnoreArray,         p.ignore_array.c_str(),  p.ignore_array.size()))  return false;
    if (!laser_.setlidaropt(LidarPropSerialBaudrate,      &p.baudrate,             sizeof(int)))            return false;
    if (!laser_.setlidaropt(LidarPropLidarType,           &p.lidar_type,           sizeof(int)))            return false;
    if (!laser_.setlidaropt(LidarPropDeviceType,          &p.device_type,          sizeof(int)))            return false;
    if (!laser_.setlidaropt(LidarPropSampleRate,          &p.sample_rate,          sizeof(int)))            return false;
    if (!laser_.setlidaropt(LidarPropAbnormalCheckCount,  &p.abnormal_check_count, sizeof(int)))            return false;
    if (!laser_.setlidaropt(LidarPropIntenstiyBit,        &p.intensity_bit,        sizeof(int)))            return false;
    if (!laser_.setlidaropt(LidarPropFixedResolution,     &p.fixed_resolution,     sizeof(bool)))           return false;
    if (!laser_.setlidaropt(LidarPropReversion,           &p.reversion,            sizeof(bool)))           return false;
    if (!laser_.setlidaropt(LidarPropInverted,            &p.inverted,             sizeof(bool)))           return false;
    if (!laser_.setlidaropt(LidarPropAutoReconnect,       &p.auto_reconnect,       sizeof(bool)))           return false;
    if (!laser_.setlidaropt(LidarPropSingleChannel,       &p.isSingleChannel,      sizeof(bool)))           return false;
    if (!laser_.setlidaropt(LidarPropIntenstiy,           &p.intensity,            sizeof(bool)))           return false;
    if (!laser_.setlidaropt(LidarPropSupportMotorDtrCtrl, &p.support_motor_dtr,    sizeof(bool)))           return false;
    if (!laser_.setlidaropt(LidarPropMaxAngle,            &p.angle_max,            sizeof(float)))          return false;
    if (!laser_.setlidaropt(LidarPropMinAngle,            &p.angle_min,            sizeof(float)))          return false;
    if (!laser_.setlidaropt(LidarPropMaxRange,            &p.range_max,            sizeof(float)))          return false;
    if (!laser_.setlidaropt(LidarPropMinRange,            &p.range_min,            sizeof(float)))          return false;
    if (!laser_.setlidaropt(LidarPropScanFrequency,       &p.frequency,            sizeof(float)))          return false;

    laser_.enableSunNoise(p.sun_noise_filter);
    laser_.enableGlassNoise(p.glass_noise_filter);

    return true;
}

  void sync_lidar_properties(LidarParams &params)
  {
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Get parameters from Lidar and updata Parameters");
    laser_.getlidaropt(LidarPropSerialPort,          &params.port[0],              params.port.size());
    laser_.getlidaropt(LidarPropIgnoreArray,         &params.ignore_array[0],      params.ignore_array.size());
    laser_.getlidaropt(LidarPropSerialBaudrate,      &params.baudrate,             sizeof(params.baudrate));
    laser_.getlidaropt(LidarPropLidarType,           &params.lidar_type,           sizeof(params.lidar_type));
    laser_.getlidaropt(LidarPropDeviceType,          &params.device_type,          sizeof(params.device_type));
    laser_.getlidaropt(LidarPropSampleRate,          &params.sample_rate,          sizeof(params.sample_rate));
    laser_.getlidaropt(LidarPropAbnormalCheckCount,  &params.abnormal_check_count, sizeof(params.abnormal_check_count));
    laser_.getlidaropt(LidarPropIntenstiyBit,        &params.intensity_bit,        sizeof(params.intensity_bit));
    laser_.getlidaropt(LidarPropFixedResolution,     &params.fixed_resolution,     sizeof(params.fixed_resolution));
    laser_.getlidaropt(LidarPropReversion,           &params.reversion,            sizeof(params.reversion));
    laser_.getlidaropt(LidarPropInverted,            &params.inverted,             sizeof(params.inverted));
    laser_.getlidaropt(LidarPropAutoReconnect,       &params.auto_reconnect,       sizeof(params.auto_reconnect));
    laser_.getlidaropt(LidarPropSingleChannel,       &params.isSingleChannel,      sizeof(params.isSingleChannel));
    laser_.getlidaropt(LidarPropIntenstiy,           &params.intensity,            sizeof(params.intensity));
    laser_.getlidaropt(LidarPropSupportMotorDtrCtrl, &params.support_motor_dtr,    sizeof(params.support_motor_dtr));
    laser_.getlidaropt(LidarPropMaxAngle,            &params.angle_max,            sizeof(params.angle_max));
    laser_.getlidaropt(LidarPropMinAngle,            &params.angle_min,            sizeof(params.angle_min));
    laser_.getlidaropt(LidarPropMaxRange,            &params.range_max,            sizeof(params.range_max));
    laser_.getlidaropt(LidarPropMinRange,            &params.range_min,            sizeof(params.range_min));
    laser_.getlidaropt(LidarPropScanFrequency,       &params.frequency,            sizeof(params.frequency));
  };

rcl_interfaces::msg::SetParametersResult 
on_param_change(const std::vector<rclcpp::Parameter> &params) 
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  LidarParams updated;
  {
      std::lock_guard<std::mutex> lock(param_mutex_);
      updated = lidar_param_;
  }

  for (const auto &p : params) {
      const std::string &name = p.get_name();
      if      (name == "port")                 updated.port                = p.as_string();
      else if (name == "ignore_array")          updated.ignore_array        = p.as_string();
      else if (name == "frame_id")              updated.frame_id            = p.as_string();
      else if (name == "baudrate")              updated.baudrate            = p.as_int();
      else if (name == "lidar_type")            updated.lidar_type          = p.as_int();
      else if (name == "device_type")           updated.device_type         = p.as_int();
      else if (name == "sample_rate")           updated.sample_rate         = p.as_int();
      else if (name == "abnormal_check_count")  updated.abnormal_check_count= p.as_int();
      else if (name == "intensity_bit")         updated.intensity_bit       = p.as_int();
      else if (name == "fixed_resolution")      updated.fixed_resolution    = p.as_bool();
      else if (name == "reversion")             updated.reversion           = p.as_bool();
      else if (name == "inverted")              updated.inverted            = p.as_bool();
      else if (name == "auto_reconnect")        updated.auto_reconnect      = p.as_bool();
      else if (name == "isSingleChannel")       updated.isSingleChannel     = p.as_bool();
      else if (name == "intensity")             updated.intensity           = p.as_bool();
      else if (name == "support_motor_dtr")     updated.support_motor_dtr   = p.as_bool();
      else if (name == "sun_noise_filter")      updated.sun_noise_filter    = p.as_bool();
      else if (name == "glass_noise_filter")    updated.glass_noise_filter  = p.as_bool();
      else if (name == "invalid_range_is_inf")  updated.invalid_range_is_inf= p.as_bool();
      else if (name == "angle_max")             updated.angle_max           = static_cast<float>(p.as_double());
      else if (name == "angle_min")             updated.angle_min           = static_cast<float>(p.as_double());
      else if (name == "range_max")             updated.range_max           = static_cast<float>(p.as_double());
      else if (name == "range_min")             updated.range_min           = static_cast<float>(p.as_double());
      else if (name == "frequency") {
          float freq = static_cast<float>(p.as_double());
          if (freq <= 0.0f) {
              RCLCPP_WARN(get_logger(), "[YDLIDAR] Invalid frequency %.2f — must be > 0", freq);
              result.successful = false;
              return result;
          }
          updated.frequency = freq;
      } else {
          RCLCPP_WARN(get_logger(), "[YDLIDAR] Unknown parameter: %s", name.c_str());
          result.successful = false;
          return result;
      }
  }

  if (!apply_properties(updated)) {
      RCLCPP_ERROR(get_logger(), "[YDLIDAR] Failed to apply updated parameters");
      result.successful = false;
      return result;
}

  if (!restart_scan()) {
      RCLCPP_ERROR(get_logger(), "[YDLIDAR] Failed to reset laser after parameter change");
      result.successful = false;
      // Laser is in an unknown state — do not restart the scan thread.
      return result;
  }

  {
      std::lock_guard<std::mutex> lock(param_mutex_);
      lidar_param_ = updated;
  }

  return result;
}


///////////////////////////////////////////////////////////////////////////
  // Laser lifecycle
///////////////////////////////////////////////////////////////////////////

// Try to connect to the laser until timeout, or max tries
bool connect_laser()
{
  bool ok = laser_.initialize();
  for (int i = 0; i < INIT_RETRIES && !ok; ++i) {
    RCLCPP_ERROR(get_logger(), "[YDLIDAR] Init failed: %s — retry %d/%d",
    laser_.DescribeError(), i + 1, INIT_RETRIES);
    std::this_thread::sleep_for(std::chrono::milliseconds(BOOT_DELAY_MS));
    ok = laser_.initialize();
  }
  if (!ok) {
      RCLCPP_FATAL(get_logger(), "[YDLIDAR] Init failed after %d retries: %s",
                    INIT_RETRIES, laser_.DescribeError());
  }
  return ok;
}

// Try to start laser until timeout, or max tries
bool start_laser()
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR] Start Laser");
  if (running_) {
    RCLCPP_WARN(get_logger(), "[YDLIDAR] Already scanning, call stop first");
    return false;
  }

  bool ok = laser_.turnOn();
  for (int i = 0; i < INIT_RETRIES && !ok; ++i) {
    RCLCPP_ERROR(get_logger(), "[YDLIDAR] Init failed: %s — retry %d/%d",
    laser_.DescribeError(), i + 1, INIT_RETRIES);
    std::this_thread::sleep_for(std::chrono::milliseconds(BOOT_DELAY_MS));
    if (!ok) {
      RCLCPP_INFO(get_logger(), "[YDLIDAR] Attempting to reconnect to laser...");
      if (!connect_laser()) {
        RCLCPP_ERROR(get_logger(), "[YDLIDAR] Reconnect failed: %s", laser_.DescribeError());
        continue;
      }
    }
    ok = laser_.turnOn();
  }
  if (!ok) {
    RCLCPP_FATAL(get_logger(), "[YDLIDAR] turnOn failed after %d retries: %s",
                  INIT_RETRIES, laser_.DescribeError());
    return false;
  }

  running_ = true;
  scan_thread_ = std::thread(&YDLidarNode::scan_loop, this);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  RCLCPP_INFO(get_logger(), "[YDLIDAR] Start Laser ended successfully!");
  return true;
}

bool stop_laser()
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR] Stop Laser");
  if (!running_) {
    RCLCPP_WARN(get_logger(), "[YDLIDAR] Already stopped, call start first");
    return false;
  }
  
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!running_) {
      RCLCPP_WARN(get_logger(), "[YDLIDAR] Already stopped, call start first");
      return false;
    }
    running_ = false;
  }
  scan_cv_.notify_all();

  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }

  bool ok = laser_.turnOff();
  if (!ok) {
    RCLCPP_FATAL(get_logger(), "[YDLIDAR] turnOff failed: %s", laser_.DescribeError());
    return false;
  }

  RCLCPP_INFO(get_logger(), "[YDLIDAR] Stop Laser ended successfully!");
  return true;
}

bool disconnect_laser()
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR] Start Disconnect");
  bool ok = laser_.disconnecting();
  if (ok){
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Disconnect ended successfuly!");
  }else{
    RCLCPP_WARN(get_logger(), "[YDLIDAR] Disconnect ended with issues!");
  }

  return ok;
}

// Stop scanning and close the serial port.
// turnOff() alone only stops the scan thread — the serial port stays open.
// disconnecting() must follow so that the next initialize() actually
// reopens the port (checkCOMMs skips reconnection if isconnected() is true).
bool stop_and_disconnect_laser()
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR] Start Stop laser and Disconnect");
  bool ok = (stop_laser() && disconnect_laser());
  if (ok){
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Stop and Disconnect ended successfuly!");
  }else{
    RCLCPP_WARN(get_logger(), "[YDLIDAR] Stop and Disconnect ended with issues!");
    return false;
  }
  return true;
}

// Open the serial port, query device health/info, then start scanning.
// Requires the port to be closed first (i.e. disconnect_laser() must have
// been called), otherwise initialize() returns early without reconnecting.
// turnOn() dereferences lidarPtr directly — initialize() must succeed first.
bool connect_and_start_laser()
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR] Start Connect and Start Laser");
  bool ok = (connect_laser() && start_laser());
  if (ok){
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Connect and Start Laser ended successfuly!");
  }else{
    RCLCPP_WARN(get_logger(), "[YDLIDAR] Connect and Start Laser ended with issues!");
    return false;
  }
  return true;
}

// Turn Off and Turn On the laser (soft reset)
bool restart_scan()
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR] Start Scan Restart");
  stop_laser();
  std::this_thread::sleep_for(std::chrono::milliseconds(RESET_DELAY_MS));
  bool ok = start_laser();
  if (ok) {
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Scan Restart ended successfully!");
  } else {
    RCLCPP_WARN(get_logger(), "[YDLIDAR] Scan Restart ended with issues!");
    return false;
  }
  return true;
}

// Turn Off laser -> disconnect and close the port -> connect to the laser -> start the laser (hard reset) 
bool reset_laser()
{
  RCLCPP_INFO(get_logger(), "[YDLIDAR] Start Reset");
  stop_and_disconnect_laser();
  std::this_thread::sleep_for(std::chrono::milliseconds(RESET_DELAY_MS));
  bool ok = connect_and_start_laser();
  if (ok){
    RCLCPP_INFO(get_logger(), "[YDLIDAR] Reset ended successfuly!");
  }else{
    RCLCPP_WARN(get_logger(), "[YDLIDAR] Reset ended with issues!");
    return false;
  }
  return true;
}


///////////////////////////////////////////////////////////////////////////
// Publishers / Services
///////////////////////////////////////////////////////////////////////////

void create_publishers()
{
  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "~/scan", rclcpp::SensorDataQoS());
  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/point_cloud", rclcpp::SensorDataQoS());
}

void create_services()
{
  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/stop_scan",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        if (stop_laser()){
          response->success = true;
          response->message = "Scan stop";
        } else {
          response->success = false;
          response->message = "Failed to stop scan";
        }
      });

  start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/start_scan",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
      {
        if (start_laser()) {
          response->success = true;
          response->message = "Scan started";
        } else {
          response->success = false;
          response->message = "Failed to start scan";
        }
      });

  restart_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/restart_scan",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
      {
        if (restart_scan()) {
          response->success = true;
          response->message = "Scan restarted";
        } else {
          response->success = false;
          response->message = "Failed to restart scan";
        }
      });

  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/reset_lidar",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        if (reset_laser()) {
          response->success = true;
          response->message = "Lidar reset Successful";
        } else {
          response->success = false;
          response->message = "Failed to reset lidar";
        }
      });
}

///////////////////////////////////////////////////////////////////////////
// Scan loop
///////////////////////////////////////////////////////////////////////////

void scan_loop()
{
  int restart_counter = 0;
  int consecutive_failures = 0;

  while (rclcpp::ok()) {

    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (!running_ || !laser_.isScanning()) {
        RCLCPP_INFO(get_logger(), "[YDLIDAR] Scan loop exiting: running=%s, isScanning=%s",
                    running_ ? "true" : "false", 
                    laser_.isScanning() ? "true" : "false");
        break;  // Exit the loop cleanly
      }
    }

    LaserScan scan;

    if (!laser_.doProcessSimple(scan)) {
      consecutive_failures++;
      RCLCPP_WARN(get_logger(),
        "[YDLIDAR] Scan failed #%d (driver error: %d (%s), scanning: %s)",
        consecutive_failures,
        static_cast<int>(laser_.getDriverError()),
        laser_.DescribeError(),
        laser_.isScanning() ? "yes" : "no");

      restart_counter++;

      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (!running_) {
          RCLCPP_INFO(get_logger(), "[YDLIDAR] Scan loop received stop signal after %d failures",
                      consecutive_failures);
          break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    consecutive_failures = 0;
    restart_counter = 0;

    std::string frame_id;
    bool invalid_range_is_inf;
    {
      std::lock_guard<std::mutex> lock(param_mutex_);
      frame_id             = lidar_param_.frame_id;
      invalid_range_is_inf = lidar_param_.invalid_range_is_inf;
    }

    auto stamp = make_stamp(scan.stamp);
    laser_pub_->publish(make_laser_scan(scan, stamp, frame_id, invalid_range_is_inf));
    pc_pub_->publish(make_point_cloud(scan, stamp, frame_id));
  }

  RCLCPP_INFO(get_logger(), "[YDLIDAR] Scan loop exited normally (restart_counter: %d, consecutive_failures: %d)", 
              restart_counter, consecutive_failures);
}


///////////////////////////////////////////////////////////////////////////
// Message builders
///////////////////////////////////////////////////////////////////////////

static builtin_interfaces::msg::Time make_stamp(uint64_t ns)
{
    builtin_interfaces::msg::Time t;
    t.sec     = static_cast<int32_t>(RCL_NS_TO_S(ns));
    t.nanosec = static_cast<uint32_t>(ns - RCL_S_TO_NS(t.sec));
    return t;
}

sensor_msgs::msg::LaserScan make_laser_scan(
    const LaserScan &scan,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id,
    bool invalid_range_is_inf)
{
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;
    msg.angle_min       = scan.config.min_angle;
    msg.angle_max       = scan.config.max_angle;
    msg.angle_increment = scan.config.angle_increment;
    msg.scan_time       = scan.config.scan_time;
    msg.time_increment  = scan.config.time_increment;
    msg.range_min       = scan.config.min_range;
    msg.range_max       = scan.config.max_range;

    if (scan.config.angle_increment <= 0.0f) {
        RCLCPP_ERROR(get_logger(), "[YDLIDAR] Invalid angle_increment: %f — skipping scan",
                      scan.config.angle_increment);
        return msg;
    }

    int size = static_cast<int>(
        (scan.config.max_angle - scan.config.min_angle) /
        scan.config.angle_increment) + 1;

    const float fill = invalid_range_is_inf
        ? std::numeric_limits<float>::infinity()
        : 0.0f;

    msg.ranges.assign(size, fill);
    msg.intensities.assign(size, 0.0f);

    for (const auto &pt : scan.points) {
        int idx = static_cast<int>(std::ceil(
            (pt.angle - scan.config.min_angle) / scan.config.angle_increment));
        if (idx < 0 || idx >= size) {
            continue;
        }
        if (pt.range >= scan.config.min_range) {
            msg.ranges[idx]      = pt.range;
            msg.intensities[idx] = pt.intensity;
        }
      }

    return msg;
}

sensor_msgs::msg::PointCloud2 make_point_cloud(
    const LaserScan &scan,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &frame_id)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;

    // Pre-filter valid points so we can allocate exactly once.
    std::vector<const LaserPoint *> valid;
    valid.reserve(scan.points.size());
    for (const auto &pt : scan.points) {
        if (pt.range >= scan.config.min_range && pt.range <= scan.config.max_range) {
            valid.push_back(&pt);
        }
    }

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(4,
        "x",         1, sensor_msgs::msg::PointField::FLOAT32,
        "y",         1, sensor_msgs::msg::PointField::FLOAT32,
        "z",         1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(valid.size());
    msg.is_dense = true;

    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(msg, "intensity");

    for (const auto *pt : valid) {
        *it_x = pt->range * std::cos(pt->angle);
        *it_y = pt->range * std::sin(pt->angle);
        *it_z = 0.0f;
        *it_i = pt->intensity;
        ++it_x; ++it_y; ++it_z; ++it_i;
    }

    return msg;
      }

  
  ///////////////////////////////////////////////////////////////////////////
    // Members
  ///////////////////////////////////////////////////////////////////////////

  CYdLidar laser_;
  LidarParams lidar_param_;
  std::mutex param_mutex_;
  std::string node_name, node_namespace;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr   laser_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          restart_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          reset_service_;
  OnSetParametersCallbackHandle::SharedPtr                    param_cb_handle_;

  std::thread       scan_thread_;
  std::atomic<bool> running_{false};
  std::condition_variable scan_cv_;
  std::mutex scan_mutex_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  std::shared_ptr<YDLidarNode> node;
  try {
      node = std::make_shared<YDLidarNode>();
  } catch (const std::exception &e) {
      RCLCPP_FATAL(rclcpp::get_logger("main"), "[YDLIDAR] Failed to start: %s", e.what());
      rclcpp::shutdown();
      return 1;
  }

  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  
  rclcpp::shutdown();
  return 0;
}