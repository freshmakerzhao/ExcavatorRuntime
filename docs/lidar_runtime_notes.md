# Airy LiDAR Runtime Notes

Date: 2026-07-06

This note records the current known-good Airy LiDAR setup for the RL excavator project.

## Scope

This setup only covers receiving and visualizing Airy LiDAR data on the host machine.
It does not send real-machine commands, PWM, or UDP control packets.

## Hardware / Network

- LiDAR model: RoboSense Airy (`RSAIRY`)
- LiDAR IP: `192.168.1.200`
- Host Ethernet interface: `enp130s0`
- Host receive IP: `192.168.1.103/24`
- Host MAC observed by LiDAR: `c4:ef:bb:7c:dc:97`

Temporary host network setup:

```bash
sudo ip link set enp130s0 up
sudo ip addr flush dev enp130s0
sudo ip addr add 192.168.1.103/24 dev enp130s0
ip -brief addr show enp130s0
ping -I 192.168.1.103 -c 3 192.168.1.200
```

## Airy Web Settings

Observed at:

```text
http://192.168.1.200
```

Current useful settings:

```text
Device IP Address:        192.168.1.200
Destination IP Address:   192.168.1.103
MSOP Port Number:         6700
DIFOP Port Number:        7789
IMU Port Number:          6688
Return Mode:              Strongest
Time Synchronization:     PTP-GPTP
Time Sync Status:         UnLock
Operation Mode:           High-Performance
Laser Status:             ON
RPM:                      599
IMU Ctrl:                 ON
IMU Output Rate:          200Hz
Frame Start Angle:        0
Dead Zone 10cm Enable:    On
Gap Filling Enable:       Off
```

Because time sync is currently `UnLock`, the runtime config uses system time instead of LiDAR clock.

## UDP Ports

Observed packets:

```text
MSOP point packets: 192.168.1.200 -> 192.168.1.103:6700, UDP length 1248
DIFOP packets:     192.168.1.200 -> 192.168.1.103:7789
IMU packets:       192.168.1.200 -> 192.168.1.103:6688, UDP length 51
```

Useful checks:

```bash
sudo timeout 5 tcpdump -eni enp130s0 -c 30 'udp and src host 192.168.1.200'
sudo timeout 5 tcpdump -vvv -ni enp130s0 -c 8 \
  'udp and src host 192.168.1.200 and (dst port 6700 or dst port 6688 or dst port 7789)'
```

## Firewall

UFW originally dropped LiDAR UDP input packets. `tcpdump` could see the packets, but Python UDP sockets
and `rslidar_sdk` could not receive them.

Temporary test rule used:

```bash
sudo iptables -I INPUT 1 -i enp130s0 -s 192.168.1.200 -d 192.168.1.103 \
  -p udp -m multiport --dports 6700,6688,7789 -j ACCEPT
```

Recommended persistent UFW rules:

```bash
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 6700
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 6688
sudo ufw allow in on enp130s0 from 192.168.1.200 to 192.168.1.103 proto udp port 7789
sudo ufw status numbered
```

Remove stale `7788` rules if present:

```bash
sudo ufw status numbered
sudo ufw delete <RULE_NUMBER_FOR_7788>
```

## ROS 2 Driver

Do not use the apt-installed `ros-jazzy-rslidar-sdk` binary for this Airy unit. That package did not
recognize `RSAIRY` in this environment.

Use the local Airy-capable SDK overlay:

```bash
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh
```

Runtime config:

```text
/home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/runtime/config_airy_current.yaml
```

Important runtime config values:

```yaml
lidar_type: RSAIRY
msop_port: 6700
difop_port: 7789
imu_port: 6688
host_address: 192.168.1.103
use_lidar_clock: false
ros_frame_id: rslidar
ros_send_point_cloud_topic: /rslidar_points
ros_send_imu_data_topic: /rslidar_imu_data
```

Do not force `wait_for_difop: false` in normal operation. With DIFOP available on `7789`, keep the
driver on the current config path and verify the startup log if DIFOP behavior needs to be checked.

Start the driver:

```bash
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:=/home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/runtime/config_airy_current.yaml
```

## Verified ROS Topics

```bash
ros2 topic list | grep rslidar
```

Observed:

```text
/rslidar_imu_data
/rslidar_points
```

Point cloud checks:

```bash
ros2 topic hz /rslidar_points
ros2 topic echo /rslidar_points --once --field header
ros2 topic echo /rslidar_points --once --field fields
```

Observed point cloud:

```text
topic: /rslidar_points
frame_id: rslidar
point type: XYZIRT
fields: x, y, z, intensity, ring, timestamp
```

The point cloud frame is the raw LiDAR frame. It is not `machine_root` or `MachineRoot`.
Any LocalMap for RRT* must first transform points from `rslidar` into `machine_root`.

## RViz2 Visualization

Start RViz2:

```bash
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh
rviz2 -d /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/rviz/airy_points.rviz
```

Expected RViz settings:

```text
Fixed Frame: machine_root
PointCloud2: /localmap/machine_root_points
MarkerArray: /occupied_cells_vis_array
MarkerArray: /localmap/reachable_workspace_markers
MarkerArray: /localmap/planned_bucket_tip_markers
Axes Reference Frame: machine_root
Grid Plane: XZ
```

`/rslidar_points` remains useful as an optional raw point cloud display, but the planning and LocalMap
chain should use `/localmap/machine_root_points`.

## Bag Recording

Record a short offline dataset:

```bash
mkdir -p /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/bags
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/bags

source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh

ros2 bag record /rslidar_points /rslidar_imu_data
```

Stop with `Ctrl+C`.

For automated short capture:

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/bags
source /opt/ros/jazzy/setup.zsh
source /home/zhaoshuai/workspace_uinty/RL_prj/AiryLidar/ros2_ws/install/setup.zsh
timeout --signal=INT 20s ros2 bag record \
  -o airy_$(date +%Y%m%d_%H%M%S) \
  /rslidar_points /rslidar_imu_data
```

## Next Development Boundary

The driver output should not be fed directly into RRT*.

Next project-side chain:

```text
/rslidar_points (frame_id=rslidar, XYZIRT)
  -> apply T_machine_root_rslidar
  -> /localmap/machine_root_points
  -> LocalMap(frame_id=machine_root)
  -> OctoMap occupied cells
  -> LocalMap.obstacles
  -> RRT bucket-tip planner constrained by shared/reachable_workspaces
  -> bucket tip waypoints_base
  -> ONNX 38d observation waypoint errors
```
