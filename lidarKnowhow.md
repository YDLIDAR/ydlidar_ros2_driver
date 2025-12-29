# YDLidar Knowhow - ROS2 Jazzy Setup Guide

This document captures all the lessons learned from setting up the YDLidar (X4 Pro / S2PRO) with ROS2 Jazzy.

---

## 📋 Hardware Information

| Property | Value |
|----------|-------|
| **Marketed Model** | YDLidar X4 Pro |
| **Detected Model (by SDK)** | S2PRO |
| **Firmware Version** | 3.1 |
| **Hardware Version** | 3 |
| **Serial Port** | `/dev/ttyUSB1` (may vary) |
| **Baudrate** | 128000 |
| **Sample Rate** | 5.00 KHz |
| **Scan Frequency** | 10.00 Hz |
| **Points per Scan** | 720 (Fixed Size) |

---

## 🔧 Key Configuration Parameters

Our lidar, although marketed as "X4 Pro", reports as **S2PRO** to the SDK, so it requires specific settings:

```yaml
# Critical parameters for our lidar (S2PRO protocol)
baudrate: 128000          # NOT 115200!
isSingleChannel: true     # MUST be true for S2PRO
lidar_type: 1             # TYPE_TRIANGLE
sample_rate: 5            # 5 KHz
reversion: true           # Required for S2PRO
support_motor_dtr: false  # Does not support DTR
```

### ⚠️ Common Mistake: `isSingleChannel`

If you set `isSingleChannel: false`, you will get this error:
```
[error] Error, cannot retrieve Lidar health code -1
[error] Failed to start scan mode -1
```
**Solution**: Set `isSingleChannel: true`

---

## 🚨 Problem: RViz2 QoS Incompatibility

### Symptom
```
[WARN] New subscription discovered on topic '/scan', requesting incompatible QoS. 
No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

### Cause
- The lidar driver publishes with **Best Effort** reliability (standard for sensor data)
- RViz2 subscribes with **Reliable** reliability by default
- These are incompatible QoS policies

### Solution
In RViz2 LaserScan display settings:
1. Expand the **Topic** section
2. Change **Reliability Policy** from `Reliable` to **`Best Effort`**
3. Set **Topic Value** to `/scan`

---

## 🚀 Quick Start Commands

### 1. Build the Package
```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select ydlidar_ros2_driver
source install/setup.bash
```

### 2. Launch the Driver (without RViz)
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### 3. Launch with RViz
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```

### 4. Check Topics
```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /scan
```

### 5. Check Serial Port
```bash
ls -la /dev/ttyUSB*
```

---

## 📁 Important Files

| File | Purpose |
|------|---------|
| `params/ydlidar.yaml` | Main configuration file |
| `config/ydlidar.rviz` | RViz2 visualization config |
| `launch/ydlidar_launch.py` | Launch driver only |
| `launch/ydlidar_launch_view.py` | Launch driver + RViz |
| `src/ydlidar_ros2_driver_node.cpp` | Driver source code |

---

## 🔄 ROS2 Jazzy Compatibility Changes

The driver was updated from ROS2 Humble to Jazzy with these changes:

### 1. Parameter Declaration API
```cpp
// OLD (Humble) - deprecated in Jazzy
node->declare_parameter("port");
node->get_parameter("port", value);

// NEW (Jazzy)
auto value = node->declare_parameter<std::string>("port", "/dev/ydlidar");
```

### 2. Service Callback Signature
```cpp
// OLD (Humble)
[](const std::shared_ptr<rmw_request_id_t> request_header,
   const std::shared_ptr<Request> req,
   std::shared_ptr<Response> response) -> bool

// NEW (Jazzy) - removed rmw_request_id_t
[](const std::shared_ptr<Request> req,
   std::shared_ptr<Response> response)
```

### 3. Launch File API
```python
# OLD (Humble)
node_executable='...'
node_name='...'
node_namespace='...'

# NEW (Jazzy)
executable='...'
name='...'
namespace='...'
```

### 4. C++ Standard
Changed from C++14 to **C++17** in CMakeLists.txt

### 5. Static Transform Publisher
```python
# OLD
arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame']

# NEW (Jazzy)
arguments=['--x', '0', '--y', '0', '--z', '0.02',
           '--roll', '0', '--pitch', '0', '--yaw', '0',
           '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']
```

---

## 🛠️ Troubleshooting

### Problem: Lidar not found
```
[error] Lidar communication error
```
**Solution**: Check the port in `ydlidar.yaml` - run `ls /dev/ttyUSB*` to find correct port

### Problem: Permission denied on serial port
```bash
sudo chmod 666 /dev/ttyUSB1
# Or add user to dialout group (permanent fix):
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Problem: RViz won't start (no display)
```
qt.qpa.xcb: could not connect to display
```
**Solution**: Run RViz in a terminal with GUI access, or use X11 forwarding if using SSH

### Problem: No laser scan visible in RViz
1. Check Topic is set to `/scan` (not `/laser_frame`)
2. Check Reliability Policy is set to `Best Effort`
3. Check Fixed Frame is set to `laser_frame`

---

## 📊 TF Frames

```
base_link
    └── laser_frame (offset: z=0.02m)
```

The static transform publisher creates the link from `base_link` to `laser_frame`.

---

## 🔗 Dependencies

- ROS2 Jazzy
- YDLidar SDK (installed at `/usr/local/lib/cmake/ydlidar_sdk`)

### Install YDLidar SDK (if not installed)
```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make
sudo make install
```

---

## 📝 Version Info

- **Driver Version**: 1.0.2-jazzy
- **SDK Version**: 1.2.19
- **ROS2 Distribution**: Jazzy
- **Last Updated**: 2025-12-29

---

## 📚 References

- [YDLidar ROS2 Driver GitHub](https://github.com/YDLIDAR/ydlidar_ros2_driver)
- [YDLidar SDK GitHub](https://github.com/YDLIDAR/YDLidar-SDK)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
