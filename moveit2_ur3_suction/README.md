# UR3e + SurfaceGripper Isaac Sim & MoveIt2 Integration

ROS2 Humble 환경에서 UR3e + Isaac Sim SurfaceGripper를 MoveIt2로 연동하는 프로젝트입니다.  
[moveit2_ur10e](../moveit2_ur10e) 프로젝트 구조를 기반으로 Robotiq 그리퍼를 SurfaceGripper(흡착 그리퍼)로 교체했습니다.

## 아키텍처

```
┌─────────────────────────────────┐
│         Isaac Sim 5.0           │
│  /isaac_joint_states  (publish) │
│  /isaac_joint_commands (subscribe)│
│  /gripper_status      (publish) │
│  /gripper_command     (subscribe)│
│  /clock               (publish) │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│    topic_based_ros2_control     │
│    (Hardware Interface)         │
└────────────┬────────────────────┘
             │ /joint_states  /follow_joint_trajectory
             ▼
┌─────────────────────────────────┐
│           MoveIt2               │
│  ur3_suction_moveit_config      │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│            RViz2                │
└─────────────────────────────────┘
```

### ur10e 대비 주요 차이점

| 항목 | moveit2_ur10e | moveit2_ur3_suction |
|------|--------------|---------------------|
| 로봇 | UR10e | UR3e |
| 그리퍼 | Robotiq 2F-140 (finger_joint 포함) | SurfaceGripper (조인트 없음) |
| MoveIt 그리퍼 컨트롤러 | `robotiq_gripper_joint_trajectory_controller` | 없음 (binary 제어) |
| 그리퍼 제어 방식 | `FollowJointTrajectory` action | `/gripper_command` 토픽 |
| SRDF 그룹 | arm + gripper | arm만 (6DOF) |
| End effector TCP | `tool0` | `suction_tcp` (tool0 기준 +50mm) |

## 패키지 구조

```
moveit2_ur3_suction/
├── README.md
├── ur3_suction_description/          # URDF 패키지
│   ├── urdf/
│   │   └── ur3_suction.urdf.xacro   # UR3e + suction_tcp 링크
│   └── config/
│       └── ur3_suction_controllers_isaac.yaml
└── ur3_suction_moveit_config/        # MoveIt2 설정 패키지
    ├── launch/
    │   └── ur3_suction_isaac_moveit.launch.py
    ├── srdf/
    │   ├── ur3_suction.srdf.xacro
    │   └── ur3_suction_macro.srdf.xacro
    └── config/
        ├── kinematics.yaml
        ├── moveit_controllers_isaac.yaml
        ├── ompl_planning.yaml
        └── initial_positions.yaml
```

## 시스템 요구사항

- Ubuntu 22.04
- ROS2 Humble
- Isaac Sim 5.0
- `topic_based_ros2_control` (PickNik): `https://github.com/PickNikRobotics/topic_based_ros2_control`

## 설치 및 빌드

```bash
# workspace 소스 폴더에 복사 (또는 symlink)
cd ~/ros2_ws/src
ln -s ~/Desktop/mygitrepos/isaac_ROS2_project/moveit2_ur3_suction/ur3_suction_description .
ln -s ~/Desktop/mygitrepos/isaac_ROS2_project/moveit2_ur3_suction/ur3_suction_moveit_config .

# ur_description (UR 공식) 필요
sudo apt install ros-humble-ur-description
# 또는 Universal_Robots_ROS2_Description을 workspace에 포함

# 빌드
cd ~/ros2_ws
colcon build --packages-select ur3_suction_description ur3_suction_moveit_config
source install/setup.bash
```

## Isaac Sim 설정

### OmniGraph 토픽 구성

| 토픽 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/isaac_joint_states` | Isaac→ROS2 | `sensor_msgs/JointState` | 관절 상태 |
| `/isaac_joint_commands` | ROS2→Isaac | `sensor_msgs/JointState` | 관절 명령 |
| `/gripper_status` | Isaac→ROS2 | `std_msgs/String` | Open/Closing/Closed |
| `/gripper_command` | ROS2→Isaac | `std_msgs/String` | open/close |
| `/clock` | Isaac→ROS2 | `rosgraph_msgs/Clock` | 시뮬레이션 시간 |

### URDF TCP 오프셋 조정

`ur3_suction.urdf.xacro`의 `suction_tcp_joint` origin을 실제 그리퍼 길이에 맞게 수정:

```xml
<origin xyz="0 0 0.05" rpy="0 0 0"/>  <!-- 0.05 → 실제 그리퍼 높이(m)로 변경 -->
```

## 실행

```bash
# 1. Isaac Sim 실행 후 Play
# 2. topic_based_ros2_control 실행 (ur10e 프로젝트의 control launch 파일 참고)

# 3. MoveIt2 + RViz 실행
ros2 launch ur3_suction_moveit_config ur3_suction_isaac_moveit.launch.py

# 4. 그리퍼 제어 (별도 터미널)
ros2 topic pub --once /gripper_command std_msgs/msg/String "data: 'close'"
ros2 topic pub --once /gripper_command std_msgs/msg/String "data: 'open'"

# 5. 현재 상태 확인
ros2 topic echo /gripper_status
ros2 topic echo /isaac_joint_states
```

## 그리퍼 제어 아키텍처

SurfaceGripper는 MoveIt trajectory 대상이 아니므로 별도 OmniGraph ScriptNode로 제어합니다.

```
/gripper_command (std_msgs/String)
    ↓
[OmniGraph: ROS2 Subscribe String]
    ↓
[ScriptNode: gripper_branch_node.py]
    "close" + status=="Open"           → Toggle
    "open"  + status in (Closed,Closing) → Toggle
    ↓
[OgnSurfaceGripper: Toggle]
```

ScriptNode 파일: `gripper_branch_node.py` (별도 레포)

## 참고 자료

- [Universal Robots ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control)
- [MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)
- [Isaac Sim ROS2 Bridge](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html)

---

**ROS2 버전:** Humble  
**Isaac Sim 버전:** 5.0  
**기반 프로젝트:** moveit2_ur10e (UR10e + Robotiq 2F-140)
