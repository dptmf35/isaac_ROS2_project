# UR10e + Robotiq 2F140 Isaac Sim & MoveIt2 Integration

ROS2 Humble 환경에서 UR10e 로봇 팔과 Robotiq 2F140 그리퍼를 Isaac Sim 4.5와 MoveIt2로 연동하는 프로젝트입니다.

## 📋 목차

- [개요](#개요)
- [시스템 요구사항](#시스템-요구사항)
- [아키텍처](#아키텍처)
- [설치 및 빌드](#설치-및-빌드)
- [실행 방법](#실행-방법)
- [트러블슈팅](#트러블슈팅)
- [주요 수정 사항](#주요-수정-사항)

## 개요

이 프로젝트는 다음을 구현합니다:
- **Isaac Sim 4.5**: 물리 시뮬레이션 및 로봇 제어
- **MoveIt2**: 모션 플래닝 및 경로 계획
- **ros2_control**: 하드웨어 인터페이스 및 컨트롤러 관리
- **topic_based_ros2_control**: Isaac Sim과 ROS2 간 토픽 기반 통신

### 주요 기능

✅ Isaac Sim의 실시간 물리 시뮬레이션
✅ MoveIt2를 통한 충돌 회피 경로 계획
✅ RViz에서 인터랙티브한 로봇 제어
✅ 시뮬레이션 시간 동기화
✅ UR10e 6DOF 매니퓰레이터 + Robotiq 2F140 그리퍼

## 시스템 요구사항

### 소프트웨어

- **OS**: Ubuntu 22.04
- **ROS**: ROS2 Humble
- **Isaac Sim**: 4.5.0
- **Python**: 3.10+
- **GPU**: NVIDIA GPU (Isaac Sim용)

### 하드웨어

- **RAM**: 최소 16GB (권장 32GB)
- **GPU**: NVIDIA RTX 시리즈 (Isaac Sim 실행용)
- **저장공간**: 최소 50GB

## 아키텍처

```
┌─────────────────┐
│   Isaac Sim     │
│   (Physics)     │
└────────┬────────┘
         │ /isaac_joint_states
         │ /isaac_joint_commands
         │ /clock
         ↓
┌─────────────────────────────┐
│  topic_based_ros2_control   │
│  (Hardware Interface)       │
└────────┬────────────────────┘
         │ /joint_states
         │ action: follow_joint_trajectory
         ↓
┌─────────────────┐
│    MoveIt2      │
│ (Motion Plan)   │
└────────┬────────┘
         │
         ↓
┌─────────────────┐
│     RViz2       │
│ (Visualization) │
└─────────────────┘
```

### 토픽 구조

| 토픽 이름 | 발행자 | 구독자 | 설명 |
|----------|--------|--------|------|
| `/isaac_joint_states` | Isaac Sim | ros2_control | 로봇의 현재 상태 (position, velocity) |
| `/isaac_joint_commands` | ros2_control | Isaac Sim | 로봇 조인트 명령 |
| `/joint_states` | joint_state_broadcaster | MoveIt2 | 표준 ROS2 joint states |
| `/clock` | Isaac Sim | All nodes | 시뮬레이션 시간 |

### Action 서버

| Action 이름 | 타입 | 제공자 | 클라이언트 |
|-------------|------|--------|-----------|
| `/joint_trajectory_controller/follow_joint_trajectory` | FollowJointTrajectory | ros2_control | MoveIt2 |
| `/robotiq_gripper_joint_trajectory_controller/follow_joint_trajectory` | FollowJointTrajectory | ros2_control | MoveIt2 |

## 설치 및 빌드

### 1. 의존성 설치

```bash
# ROS2 Humble 기본 패키지 (설치되어 있다고 가정)
sudo apt update

# MoveIt2 설치
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-ros-planning-interface

# UR 로봇 관련 패키지
sudo apt install -y \
    ros-humble-ur-msgs \
    ros-humble-ur-robot-driver \
    ros-humble-ur-controllers

# topic_based_ros2_control (핵심!)
sudo apt install -y ros-humble-topic-based-ros2-control

# 기타 의존성
sudo apt install -y \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers
```

### 2. 워크스페이스 빌드

```bash
cd ~/samsung_ws

# 전체 빌드
colcon build --symlink-install

# 빌드 환경 source
source install/setup.bash
```

### 3. 빌드 확인

빌드가 성공하면 다음 패키지들이 설치됩니다:

```bash
# 빌드된 패키지 확인
ls install/

# 주요 패키지:
# - ur_description
# - ur_robot_driver
# - ur_robotiq_description
# - ur_robotiq_moveit_config
# - robotiq_description
# - robotiq_controllers
```

## 실행 방법

### 사전 준비

1. **Isaac Sim 실행 및 준비**
   - Isaac Sim 4.5.0 실행
   - UR10e + Robotiq 2F140 씬 로드
   - ROS2 Bridge 활성화
   - Action Graph에서 다음 확인:
     - ROS2 Publish Clock 노드 활성화
     - ROS2 Publish JointState 노드: `/isaac_joint_states` 토픽으로 발행
     - ROS2 Subscribe JointState 노드: `/isaac_joint_commands` 토픽 구독
   - **Play 버튼 클릭하여 시뮬레이션 시작**

2. **토픽 확인**

```bash
# 새 터미널에서
source ~/samsung_ws/install/setup.bash

# Isaac Sim 토픽 발행 확인
ros2 topic list | grep isaac
# 출력 예상:
# /isaac_joint_states
# /isaac_joint_commands
# /isaac_gripper_state

# joint states 확인
ros2 topic echo /isaac_joint_states --once
```

### 실행 순서

#### Terminal 1: ros2_control 시작

```bash
cd ~/samsung_ws
source install/setup.bash

# ros2_control 노드 실행
ros2 launch ur_robotiq_description ur_robotiq_isaac_control.launch.py \
    sim_isaac:=true \
    use_sim_time:=true
```

**예상 출력:**
```
[INFO] [resource_manager]: Successful initialization of hardware 'ur10e'
[INFO] [resource_manager]: Successful initialization of hardware 'gripper'
[INFO] [controller_manager]: update rate is 500 Hz
[INFO] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[INFO] [spawner_joint_trajectory_controller]: Configured and activated joint_trajectory_controller
```

**중요**: 다음 메시지가 나타나면 성공:
- `Successful 'activate' of hardware 'ur10e'`
- `Successful 'activate' of hardware 'gripper'`
- `joint_trajectory_controller` 활성화됨

#### Terminal 2: MoveIt2 및 RViz 시작

```bash
cd ~/samsung_ws
source install/setup.bash

# MoveIt2 실행
ros2 launch ur_robotiq_moveit_config ur_robotiq_isaac_moveit.launch.py \
    use_fake_hardware:=true \
    use_sim_time:=true \
    launch_rviz:=true
```

**예상 출력:**
```
[move_group-1] You can start planning now!
```

**RViz가 자동으로 실행됩니다.**

### RViz에서 제어하기

1. **로봇 상태 확인**
   - RViz 왼쪽 패널에서 로봇이 Isaac Sim과 동일한 자세로 표시되는지 확인
   - Isaac Sim에서 로봇을 움직이면 RViz에서도 따라 움직여야 함

2. **모션 플래닝**
   - RViz 하단 "MotionPlanning" 패널 사용
   - "Planning" 탭 선택
   - Interactive Marker (파란색/녹색 화살표)를 드래그하여 목표 위치 설정
   - 또는 "Goal State" 드롭다운에서 사전 정의된 자세 선택

3. **계획 및 실행**
   - **Plan** 버튼 클릭 → 경로가 계획됨 (주황색으로 표시)
   - 경로가 만족스러우면 **Execute** 버튼 클릭
   - Isaac Sim에서 로봇이 계획된 경로를 따라 움직임

### 그리퍼 제어 (수동)

```bash
# 그리퍼 열기 (position = 0.0)
ros2 action send_goal /robotiq_gripper_joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{
      trajectory: {
        joint_names: ['finger_joint'],
        points: [
          { positions: [0.0], time_from_start: { sec: 2, nanosec: 0 } }
        ]
      }
    }"

# 그리퍼 닫기 (position = 0.5)
ros2 action send_goal /robotiq_gripper_joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{
      trajectory: {
        joint_names: ['finger_joint'],
        points: [
          { positions: [0.5], time_from_start: { sec: 2, nanosec: 0 } }
        ]
      }
    }"
```

## 트러블슈팅

### 1. "Didn't receive robot state with recent timestamp" 에러

**증상:**
```
[WARN] Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds.
Check clock synchronization!
```

**원인:** Isaac Sim과 ROS2 노드 간 시간 동기화 문제

**해결:**
1. Isaac Sim에서 ROS2 Publish Clock이 활성화되어 있는지 확인
2. 모든 launch 파일에 `use_sim_time:=true` 사용
3. 두 터미널 모두 재시작

### 2. Action server not available

**증상:**
```
[ERROR] Action client not connected to action server: joint_trajectory_controller/follow_joint_trajectory
```

**원인:** ros2_control_node가 실행되지 않았거나 컨트롤러가 로드되지 않음

**해결:**
```bash
# 컨트롤러 상태 확인
ros2 control list_controllers

# 예상 출력:
# joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

# Action 서버 확인
ros2 action list
# /joint_trajectory_controller/follow_joint_trajectory 가 있어야 함
```

### 3. Isaac Sim 토픽이 발행되지 않음

**증상:**
```bash
ros2 topic list | grep isaac
# 아무것도 출력되지 않음
```

**해결:**
1. Isaac Sim에서 **Play** 버튼을 눌렀는지 확인
2. Action Graph가 제대로 설정되었는지 확인
3. ROS2 Bridge 확장이 활성화되었는지 확인

### 4. Planning succeeds but execution fails

**증상:**
```
[INFO] Motion plan was computed successfully.
[WARN] Failed to validate trajectory
[INFO] Execution completed: ABORTED
```

**해결:**
1. `/joint_states` 토픽이 발행되는지 확인:
   ```bash
   ros2 topic hz /joint_states
   ```
2. joint_state_broadcaster가 활성화되었는지 확인:
   ```bash
   ros2 control list_controllers | grep joint_state_broadcaster
   ```

### 5. 빌드 에러

**robotiq_driver 에러:**
```
Failed   <<< robotiq_driver [exited with code 1]
```

**해결:** robotiq_driver는 실제 하드웨어용이므로 삭제했습니다. 이미 제거되어 있어야 합니다.

## 주요 수정 사항

이 프로젝트를 위해 다음 파일들이 수정되었습니다:

### 1. URDF/ros2_control 설정

**파일:** `Universal_Robots_ROS2_Description/urdf/isaac_ur.ros2_control.xacro`

**수정 내용:**
- `joint_states_topic`: `/joint_states` → `/isaac_joint_states`
- Isaac Sim의 토픽과 일치하도록 수정

```xml
<param name="joint_states_topic">/isaac_joint_states</param>
```

**파일:** `ros2_robotiq_gripper/robotiq_description/urdf/2f_140.ros2_control.xacro`

**수정 내용:**
- 기본값 `isaac_joint_states`: `/joint_states` → `/isaac_joint_states`

### 2. Controller 설정

**파일:** `ur_robotiq_description/config/ur_robotiq_controllers_isaac.yaml`

**추가 내용:**
```yaml
controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

### 3. Launch 파일 - ros2_control

**파일:** `ur_robotiq_description/launch/ur_robotiq_isaac_control.launch.py`

**수정 내용:**

1. `ros2_control_node`에 `use_sim_time` 추가:
```python
control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        robot_description,
        update_rate_config_file,
        ParameterFile(initial_controllers, allow_substs=True),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},  # 추가
    ],
    output="screen",
)
```

2. `robot_state_publisher`에 `use_sim_time` 추가:
```python
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[
        robot_description,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},  # 추가
    ],
    output="both",
)
```

3. `controller_spawner` 함수에 `use_sim_time` 추가:
```python
def controller_spawner(name, active=True):
    args = [
        name,
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", LaunchConfiguration("controller_spawner_timeout"),
    ]
    if not active:
        args.append("--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],  # 추가
        output="screen"
    )
```

### 4. Launch 파일 - MoveIt2

**파일:** `ur_robotiq_moveit_config/launch/ur_robotiq_isaac_moveit.launch.py`

**수정 내용:**

`move_group_node`에 topic remapping 추가:
```python
move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        {"use_sim_time": use_sim_time},
        warehouse_ros_config,
    ],
    remappings=[
        ("/joint_states", "/isaac_joint_states"),  # 추가
    ],
)
```

## 테스트된 시나리오

✅ Isaac Sim에서 로봇 상태 읽기
✅ MoveIt2를 통한 경로 계획
✅ 계획된 경로를 Isaac Sim에서 실행
✅ RViz Interactive Marker를 통한 목표 설정
✅ 그리퍼 개폐 제어
✅ 시뮬레이션 시간 동기화

## 향후 작업

- [ ] Pick and Place 구현
- [ ] Cartesian Path Planning
- [ ] Collision Object 추가
- [ ] Vision 기반 객체 인식 연동
- [ ] MoveIt Servo (실시간 제어)
- [ ] 멀티 로봇 협업

## 참고 자료

- [Universal Robots ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [ros2_robotiq_gripper](https://github.com/robotiq/ros2_robotiq_gripper)
- [topic_based_ros2_control](https://github.com/PickNikRobotics/topic_based_ros2_control)
- [MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)

## 라이선스

이 프로젝트는 다양한 오픈소스 패키지들을 통합한 것입니다. 각 패키지의 라이선스를 참조하세요.

## 기여

프로젝트 관련 문의나 개선 사항은 이슈로 등록해주세요.

---

**작성일:** 2025-10-28
**ROS2 버전:** Humble
**Isaac Sim 버전:** 4.5.0
**테스트 환경:** Ubuntu 22.04
