# Fixed-Wing Autonomous Flight

## 개발 환경

- PX4 SITL
- Gazebo Classic
- ROS 2 Humble
- QGroundControl
- Python
- Micro XRCE-DDS

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

### 5. QGroundControl 연결

- PX4 SITL 자동 연결
- Manual Takeoff
- RC 또는 QGC에서 Offboard 모드 전환

---

## 구현 완료

- [x] PX4 SITL - QGroundControl 연결
- [x] PX4 SITL - ROS 2 (Micro XRCE-DDS) 통신
- [x] ROS 2 기반 Offboard 제어
- [x] 수동 → Offboard → 수동 전환
- [x] NED Waypoint 비행
- [x] Waypoint 통과 판정 로직
- [x] Mission State Machine

## 개발 예정
- [ ] YOLO 객체 인식 모델 학습 및 테스트
- [ ] 실제 PX4 Flight Controller 적용
- [ ] Raspberry Pi 연동
- [ ] 실기체 Offboard 자동비행
- [ ] Raspberry Pi 기반 YOLO 추론
- [ ] YOLO 기반 경로 분기
- [ ] 통합 자율비행 시험
