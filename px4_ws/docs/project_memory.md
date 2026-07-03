# 项目长期记忆：PX4 DDS + NOKOV

更新时间：2026-07-01

这份文档用于记录最近实验中已经确认的稳定事实。后续继续调试时，优先以这里的内容为准，避免反复重新判断工作空间、话题、启动顺序和安全边界。

## 1. 当前实验环境

- 平台：OrangePi 5 Plus
- 系统：Ubuntu 20.04
- ROS 版本：ROS 2 Foxy
- 飞控：PX4 实机
- 通信：Micro XRCE-DDS
- 动捕：NOKOV
- 当前场景：真实硬件实验，不是仿真环境

OrangePi 实机端使用的 PX4 DDS 话题没有 `_v1`、`_v4` 这类版本后缀。不要套用 Humble 仿真里的话题名。

常用 PX4 DDS 话题：

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_mocap_odometry
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
```

## 2. 工作空间分工

### 2.1 `~/px4_ws`

这是当前 PX4 DDS 主工作空间。

主要负责：

```text
1. PX4 DDS 消息 px4_msgs
2. NOKOV 到 PX4 的 mocap_to_px4_bridge
3. DDS Offboard 控制 dds_offboard_control
4. 与 /fmu/in、/fmu/out 相关的 ROS 2 节点
```

重要包：

```text
~/px4_ws/src/px4_msgs
~/px4_ws/src/mocap_to_px4_bridge
~/px4_ws/src/dds_offboard_control
```

### 2.2 `~/px4nokov_ws`

这个工作空间不是完全不用。它仍然负责真实 NOKOV 节点。

重要包：

```text
~/px4nokov_ws/src/mocap_nokov
```

启动真实 NOKOV 节点时必须 source 这个工作空间：

```bash
source /opt/ros/foxy/setup.bash
source ~/px4nokov_ws/install/setup.bash
ros2 launch mocap_nokov mocap.launch.py
```

一句话分工：

```text
px4nokov_ws：负责真实 NOKOV 节点，输出 /Robot_1/pose
px4_ws：负责 PX4 DDS、bridge、Offboard 控制
```

以后不要简单说 `px4nokov_ws` 是不用的老工作区。更准确的说法是：它不是 DDS 控制主工作区，但它仍然是当前真实 NOKOV 数据源。

## 3. 正常启动顺序

### 终端 1：启动 Micro XRCE-DDS Agent

优先使用脚本：

```bash
~/start_px4_dds.sh
```

脚本内容当前等价于：

```bash
source /opt/ros/foxy/setup.bash
source ~/px4_ws/install/setup.bash

MicroXRCEAgent serial --dev /dev/ttyS6 -b 921600
```

如果 `/dev/ttyS6` 不通，再考虑：

```bash
MicroXRCEAgent serial --dev /dev/ttyS9 -b 921600
```

这个终端保持运行，不要关闭。

### 终端 2：检查 PX4 DDS 输出

```bash
source /opt/ros/foxy/setup.bash
source ~/px4_ws/install/setup.bash

ros2 topic list | grep fmu
ros2 topic hz /fmu/out/vehicle_status
ros2 topic hz /fmu/out/vehicle_odometry
```

期望：

```text
/fmu/out/vehicle_status 有输出
/fmu/out/vehicle_odometry 有输出
```

如果没有 `/fmu/out/...`，优先检查：

```text
1. MicroXRCEAgent 是否还在运行
2. 串口是否正确，/dev/ttyS6 或 /dev/ttyS9
3. PX4 里的 uxrce_dds_client 状态
4. TELEM2 接线和波特率
```

PX4 Shell 中可检查：

```sh
uxrce_dds_client status
```

### 终端 3：启动 NOKOV 节点

```bash
source /opt/ros/foxy/setup.bash
source ~/px4nokov_ws/install/setup.bash

ros2 launch mocap_nokov mocap.launch.py
```

### 终端 4：检查真实 NOKOV 位姿

```bash
source /opt/ros/foxy/setup.bash
source ~/px4nokov_ws/install/setup.bash

ros2 topic list | grep Robot
ros2 topic hz /Robot_1/pose
timeout 3s ros2 topic echo /Robot_1/pose
```

通过标准：

```text
1. /Robot_1/pose 存在
2. 频率稳定
3. 手拿刚体移动时 position 连续变化
4. orientation 存在并连续
5. 刚体 ID 不乱跳、不丢失
6. 不出现 9999.999
```

### 终端 5：启动 mocap_to_px4_bridge

```bash
source /opt/ros/foxy/setup.bash
source ~/px4_ws/install/setup.bash

ros2 run mocap_to_px4_bridge mocap_bridge
```

### 终端 6：检查 PX4 外部动捕输入和 EKF 输出

ROS 2 Foxy 不支持 `ros2 topic echo --once`，用 `timeout`。

```bash
source /opt/ros/foxy/setup.bash
source ~/px4_ws/install/setup.bash

timeout 3s ros2 topic echo /fmu/in/vehicle_mocap_odometry
timeout 3s ros2 topic echo /fmu/out/vehicle_odometry
timeout 5s ros2 topic echo /fmu/out/vehicle_local_position | grep -E "heading|heading_good_for_control|xy_valid|z_valid|dead_reckoning"
```

重点看：

```text
/fmu/in/vehicle_mocap_odometry 中 q 不是 NaN
/fmu/in/vehicle_mocap_odometry 中 orientation_variance 不是 NaN
/fmu/out/vehicle_odometry 中 q 不是 NaN
/fmu/out/vehicle_local_position 中 xy_valid=true
/fmu/out/vehicle_local_position 中 z_valid=true
/fmu/out/vehicle_local_position 中 dead_reckoning=false
heading_good_for_control 尽量变为 true
```

## 4. NOKOV 话题事实

真实动捕重点使用：

```text
/Robot_1/pose
```

类型：

```text
geometry_msgs/msg/PoseStamped
```

该话题已经确认包含三维位置和四元数姿态。

示例 orientation：

```text
x: 0.039
y: 0.058
z: -0.040
w: 0.996
```

另外存在：

```text
/Robot_1/ground_pose
```

类型：

```text
geometry_msgs/msg/Pose2D
```

`ground_pose` 只有平面位姿，不适合作为 PX4 三维外部定位融合的优先输入。

## 5. mocap_to_px4_bridge 记忆

源码：

```bash
~/px4_ws/src/mocap_to_px4_bridge/src/mocap_bridge.cpp
```

当前目标默认发布话题：

```text
/fmu/in/vehicle_mocap_odometry
```

旧记录中曾经使用或提到：

```text
/fmu/in/vehicle_visual_odometry
```

当前排查和后续实验优先使用：

```text
/fmu/in/vehicle_mocap_odometry
```

### 5.1 位置转换

当前假设：

```text
NOKOV / ROS：x 前，y 左，z 上
PX4 NED：   x 前，y 右，z 下
```

因此位置转换为：

```text
PX4 x =  NOKOV x
PX4 y = -NOKOV y
PX4 z = -NOKOV z
```

代码逻辑：

```cpp
odom.position[0] = raw_x;
odom.position[1] = invert_y_ ? -raw_y : raw_y;
odom.position[2] = invert_z_ ? -raw_z : raw_z;
```

### 5.2 姿态转换

ROS `PoseStamped` orientation 顺序：

```text
x, y, z, w
```

PX4 `VehicleOdometry.q` 顺序：

```text
w, x, y, z
```

当前第一版最小转换：

```cpp
odom.q[0] = qw;
odom.q[1] = qx;
odom.q[2] = invert_y_ ? -qy : qy;
odom.q[3] = invert_z_ ? -qz : qz;
```

要求：

```text
1. 写入前先检查四元数 norm
2. norm 有效并大于 1e-6 时归一化
3. 如果 odom.q[0] < 0，则四元数整体取反
4. q 有效时 orientation_variance 写 0.05
5. q 无效时 q 和 orientation_variance 继续写 NaN，并打印 throttle warning
```

有效姿态方差：

```cpp
odom.orientation_variance[0] = 0.05f;
odom.orientation_variance[1] = 0.05f;
odom.orientation_variance[2] = 0.05f;
```

### 5.3 NOKOV 无效值过滤

NOKOV 丢刚体时可能输出：

```text
x=9999.999
y=9999.999
z=9999.999
```

bridge 应过滤：

```text
NaN / inf：跳过发布
abs(x) > 10 m：跳过发布
abs(y) > 10 m：跳过发布
abs(z) > 5 m：跳过发布
```

过滤 9999 只能防止错误坐标进入 PX4，不能解决无人机飞起来后刚体丢失的问题。

## 6. DDS Offboard 节点记忆

包：

```bash
~/px4_ws/src/dds_offboard_control
```

源码：

```bash
~/px4_ws/src/dds_offboard_control/src/dds_offboard_control_node.cpp
```

节点：

```text
dds_offboard_control_node
```

发布：

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

订阅：

```text
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status
```

当前已确认的正确 Offboard setpoint：

```text
x 固定为进入控制时的 hold_x
y 固定为进入控制时的 hold_y
z = -0.30
yaw = NaN
yawspeed = NaN
```

推荐带桨前后的运行命令：

```bash
ros2 run dds_offboard_control dds_offboard_control_node --ros-args \
  -p target_z:=-0.30 \
  -p auto_offboard:=true \
  -p auto_arm:=false
```

带桨测试不建议使用：

```text
auto_arm:=true
```

无桨地面验证中曾经验证过：

```text
1. /fmu/in/offboard_control_mode 约 20 Hz
2. /fmu/in/trajectory_setpoint 约 20 Hz
3. QGC 可进入 Offboard
4. 无桨可 Arm
5. 可立即 Disarm / Kill
```

## 7. PX4 / EKF2 记忆

此前成功位置融合时使用过：

```text
EKF2_EV_CTRL = 3
EKF2_HGT_REF = 3
EKF2_GPS_CTRL = 0
```

含义：

```text
EKF2_EV_CTRL=3：只融合外部视觉水平位置 + 外部视觉高度
EKF2_HGT_REF=3：高度参考使用 Vision
EKF2_GPS_CTRL=0：室内关闭 GPS 融合
```

此前 yaw 对齐问题通过启用磁罗盘解决过：

```text
SYS_HAS_MAG = 1
EKF2_MAG_TYPE = 0
```

理想 estimator 状态：

```text
cs_tilt_align: True
cs_yaw_align: True
cs_mag_hdg: True
cs_ev_pos: True
cs_ev_hgt: True
cs_inertial_dead_reckoning: False
cs_fake_pos: False
```

理想 local position 状态：

```text
xy_valid: true
z_valid: true
v_xy_valid: true
v_z_valid: true
dead_reckoning: false
```

当前继续带桨 Offboard 前的重要门槛：

```text
heading_good_for_control 应该变为 true，或至少其 false 的原因必须被理解和验证。
```

## 8. 已经遇到过的重要问题

### 8.1 NOKOV 丢刚体输出 9999

现象：

```text
x=9999.999
y=9999.999
z=9999.999
```

判断：

```text
根因不是 DDS，也不是 PX4，而是 NOKOV 刚体追踪丢失。
```

可能原因：

```text
1. 反光球被机体、电机、桨叶、支架遮挡
2. 反光球布局太对称
3. 反光球固定不牢
4. 起飞后姿态变化导致 marker 数量不足
5. 动捕区域边缘或主机附近存在相机覆盖死角
```

当前软件处理：

```text
mocap_to_px4_bridge 已过滤 9999，不再把 9999 发布给 PX4。
```

但物理问题仍需解决：

```text
需要重新设计并验证无人机反光球刚体。
```

### 8.2 heading_good_for_control=false

此前低高度测试现象：

```text
1. 飞机能起升
2. 高度大多在 z=-0.13 到 -0.15 附近
3. XY 快速漂移
4. xy_valid=true
5. z_valid=true
6. dead_reckoning=false
7. heading_good_for_control=false
```

判断：

```text
PX4 并不是看不到位置。
PX4 能看到 XY 漂移。
问题更像是 heading 不适合控制，导致水平位置控制无法正确修正。
```

之前关键原因：

```text
NOKOV 的 orientation 没有写入 PX4 VehicleOdometry.q，导致 /fmu/out/vehicle_odometry 中 q 为 NaN。
```

因此优先修复对象是：

```bash
~/px4_ws/src/mocap_to_px4_bridge/src/mocap_bridge.cpp
```

不是优先修改 Offboard 节点。

### 8.3 Offboard 漂移

带桨 Offboard 低高度尝试中出现过漂移。

当时 bridge 终端出现：

```text
Invalid mocap data: out of range x=9999.999 y=9999.999 z=9999.999, skip publish
```

判断：

```text
飞起来后 NOKOV 刚体追踪不稳定，动捕定位中断。
Offboard 节点仍继续发布目标点。
无人机因此可能漂移。
```

后续需要给 Offboard 节点加定位丢失 failsafe。

## 9. 安全停止点

### 9.1 进入 EKF2 外部定位融合前

必须满足：

```text
1. 真实 /Robot_1/pose 稳定
2. /fmu/in/vehicle_mocap_odometry 稳定
3. 手拿无人机 x/y/z 方向检查通过
4. 没有明显跳变
5. 刚体 ID 不丢失
```

### 9.2 进入 Position 悬停前

必须满足：

```text
1. EKF2 参数已配置，必要时飞控已重启
2. /fmu/out/vehicle_local_position 有效
3. xy_valid 和 z_valid 正常
4. 手拿无人机检查 local position 方向正确
5. 遥控器接管和 Kill switch 有效
```

### 9.3 进入 Offboard 悬停前

必须满足：

```text
1. Position 短悬停已通过，或地面 local position 验证完全干净
2. /fmu/in/offboard_control_mode 持续发布
3. /fmu/in/trajectory_setpoint 持续发布
4. 目标点在当前位置附近，不给大范围轨迹
5. 遥控器可随时切回安全模式
6. 已准备 rosbag2 记录
7. heading / 姿态 / 定位有效性问题已解决或明确验证
```

在 `heading_good_for_control` 问题解决前，不进行带桨 Offboard 悬停测试。

## 10. 无桨手拿测试

运行：

```bash
source /opt/ros/foxy/setup.bash
source ~/px4_ws/install/setup.bash

timeout 5s ros2 topic echo /fmu/out/vehicle_local_position | grep -E "x:|y:|z:|heading:|heading_good_for_control"
```

手拿飞机执行：

```text
1. 向机头方向移动约 20 cm
2. 向机体右侧移动约 20 cm
3. 向上移动约 20 cm
4. 原地顺时针转动约 30 度
```

记录：

```text
向前移动时 x 如何变化
向右移动时 y 如何变化
向上移动时 z 是否变负
顺时针转动时 heading 是否连续变化
heading_good_for_control 是否变 true
```

只有确认坐标方向和航向方向基本合理后，才继续低高度带桨测试。

## 11. 反光球刚体改进建议

飞行状态下 NOKOV 刚体追踪丢失是当前重要风险。

建议：

```text
1. 至少 4 个反光球，建议 5 个
2. 不要共线
3. 不要完全对称
4. 不要全部贴在同一低平面
5. 尽量加高支架，避免被机体和桨叶遮挡
6. 固定必须足够硬，避免飞行震动导致 marker 相对位置变化
7. 刚体原点尽量靠近飞控 IMU 或无人机质心
```

推荐布局：

```text
前方 1 个
左后 1 个
右后 1 个
中部偏左或偏右 1 个
上方或加高支架再加 1 个
```

手拿刚体可靠性测试：

```text
前后左右平移
上下移动
yaw 左右旋转
roll 倾斜 10 到 20 度
pitch 倾斜 10 到 20 度
抬高到 30 cm、50 cm、1 m
移动到实际飞行区域边缘
```

通过标准：

```text
全程不出现 9999
频率稳定
位置连续
刚体 ID 不跳
NOKOV 软件中刚体不闪烁、不丢失
```

## 12. Offboard failsafe 后续需求

后续 `dds_offboard_control` 应增加定位丢失保护：

```text
1. 检查 local_position 是否 xy_valid/z_valid
2. 检查 dead_reckoning 是否 false
3. 检查 vehicle_mocap_odometry 或 local_position 是否超时
4. 定位无效时拒绝自动 Arm
5. 飞行中定位失效时发送降落、切 Position 或触发安全逻辑
6. 带桨测试时 auto_arm 默认 false
```

## 13. rosbag2 记录建议

单机动捕融合阶段：

```bash
ros2 bag record \
  /Robot_1/pose \
  /fmu/in/vehicle_mocap_odometry \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_status
```

Offboard 阶段增加：

```bash
ros2 bag record \
  /Robot_1/pose \
  /fmu/in/vehicle_mocap_odometry \
  /fmu/in/offboard_control_mode \
  /fmu/in/trajectory_setpoint \
  /fmu/in/vehicle_command \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_status
```

建议目录命名：

```text
bags/YYYYMMDD_single_mocap_fusion_test_01
bags/YYYYMMDD_position_hover_test_01
bags/YYYYMMDD_offboard_hover_test_01
```

## 14. 常用验证命令

检查 bridge 输入 PX4 的动捕数据：

```bash
timeout 3s ros2 topic echo /fmu/in/vehicle_mocap_odometry
```

检查 PX4 输出 odometry：

```bash
timeout 3s ros2 topic echo /fmu/out/vehicle_odometry
```

检查 local position 和 heading：

```bash
timeout 5s ros2 topic echo /fmu/out/vehicle_local_position | grep -E "heading|heading_good_for_control|xy_valid|z_valid|dead_reckoning"
```

检查 Offboard setpoint：

```bash
timeout 3s ros2 topic echo /fmu/in/trajectory_setpoint
```

检查话题频率：

```bash
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
ros2 topic hz /Robot_1/pose
```

## 15. 当前后续优先级

### 第一优先级：验证 mocap_bridge 姿态输入

目标：

```text
/fmu/in/vehicle_mocap_odometry q 不再是 NaN
/fmu/in/vehicle_mocap_odometry orientation_variance 不再是 NaN
/fmu/out/vehicle_odometry q 不再是 NaN
heading_good_for_control 变 true，或明确知道为什么仍为 false
```

### 第二优先级：无桨手拿验证

目标：

```text
x/y/z 方向正确
heading 连续变化
没有 9999
local_position 不跳变
```

### 第三优先级：提升 NOKOV 刚体可靠性

目标：

```text
飞行姿态变化时刚体不丢失
飞行区域内不出现 9999
```

### 第四优先级：Offboard failsafe

目标：

```text
定位无效时不自动 Arm
飞行中定位丢失时不继续盲目发悬停目标
```

## 16. 当前阶段一句话总结

当前系统已经完成 PX4 DDS 通信、NOKOV 位姿输入、bridge 转换、Offboard XY/Z setpoint 固定发布。真实 NOKOV 数据来自 `px4nokov_ws` 的 `mocap_nokov`，PX4 DDS、bridge 和 Offboard 控制在 `px4_ws`。下一步重点是验证 `/Robot_1/pose` 的 orientation 已正确写入 `/fmu/in/vehicle_mocap_odometry`，让 PX4 的 odometry q 有效，并在无桨手拿测试通过后，再考虑低高度带桨测试。
