# Fixed-Wing Autonomous Flight

## 개발 환경

- PX4 v1.18.0-beta1
- px4_msgs (PX4 v1.18 대응)
- Gazebo Classic
- ROS 2 Humble
- QGroundControl
- Python 3
- Micro XRCE-DDS

---

## 실행 방법

### 1. PX4 SITL 실행

```bash
make px4_sitl gazebo
```

### 2. Micro XRCE-DDS Agent 실행

```bash
MicroXRCEAgent udp4 -p 8888
```

### 3. ROS 2 Workspace 빌드

```bash
cd ~/Aircraft_ws
colcon build
source install/setup.bash
```

### 4. 자동비행 노드 실행

```bash
ros2 run fixedwing_autonomy fixedwing_offboard_node
```

### 5. (시험용) 게이트 선택 노드 실행

```bash
ros2 run aircraft_vision gate_decision_node
```

### 6. QGroundControl 연결

- PX4 SITL 자동 연결
- Manual Takeoff
- RC 또는 QGC에서 Offboard 모드 전환

---

## 구현 완료

- [x] PX4 SITL - QGroundControl 연결
- [x] PX4 SITL - ROS 2 (Micro XRCE-DDS) 통신
- [x] ROS 2 기반 Fixed-Wing Offboard 제어
- [x] 수동 → Offboard → 수동 전환
- [x] NED Waypoint 자동비행
- [x] Waypoint 통과 판정 로직
- [x] Mission State Machine
- [x] Airspeed 유지 제어
- [x] AGL 기반 고도 제어
- [x] Dynamic Gate Waypoint 생성
- [x] Gate Approach / Center / Exit 경로 생성
- [x] 이미지 선택 기반 경로 변경 (Tank / Building)
- [x] Gate 통과 성공/실패 판정
- [x] ROS 2 Topic 기반 노드 간 통신

---

## 개발 예정

- [ ] YOLOX 객체 인식 모델 고도화
- [ ] Jetson Orin Nano 연동
- [ ] 카메라 영상 기반 Gate 인식
- [ ] Gate 위치 추정 (Bounding Box → Local NED)
- [ ] Gate 좌표 실시간 생성
- [ ] 영상 인식 실패 시 Manual 재진입 로직
- [ ] 실기체 Offboard 비행 검증
- [ ] 실기체 Gate 통과 시험
