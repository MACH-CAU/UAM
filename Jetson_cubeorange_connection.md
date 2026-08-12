# Cube Orange ↔ Jetson Orin Nano Micro XRCE-DDS 직접 연결 실행 방법

## 1. 최종 연결 구조

현재 Raspberry Pi는 사용하지 않고 **Cube Orange와 Jetson Orin Nano를 직접 연결**한다.

```text
RC 조종기(조종기 ch6 안 먹으면 FC 재부팅 한 번 ㄱㄱ)
    ↓
Cube Orange
    ↕
TELEM2
    ↕ UART
UART-to-USB Converter
    ↕ USB
Jetson Orin Nano
    ↓
MicroXRCEAgent
    ↓
ROS2 Humble
    ↓
Aircraft_ws
```

현재 통신 속도:

```text
921600 baud
```

현재 확인된 상태:

```text
Cube ↔ Jetson Micro XRCE-DDS 연결    성공
/fmu/out/... 토픽 discovery          성공
VehicleStatus 실제 echo              성공
Jetson → FC topic publish            성공
```

---

# 2. 하드웨어 연결

Cube Orange의 `TELEM2`와 UART-to-USB Converter를 연결하고,
Converter의 USB를 Jetson에 연결한다.

현재 사용 중인 USB serial device는 보통:

```text
/dev/ttyUSB0
```

이다.

Jetson에서 확인:

```bash
ls -l /dev/ttyUSB*
```

또는:

```bash
ls -l /dev/serial/by-id/
```

`/dev/ttyUSB0`가 확인되면 이후 Agent 명령에서 해당 포트를 사용한다.

---

# 3. UART-to-USB Converter LED가 꺼졌을 때

현재 사용 중인 Converter에서 간헐적으로 LED가 꺼지거나 conneted가 안 될 때

이 경우:

```text
1. Converter의 6핀 커넥터 쪽을 뺀다.
2. 다시 6핀 커넥터를 꽂는다.
3. Converter LED가 다시 들어오는지 확인한다.
4. Jetson에서 /dev/ttyUSB0가 존재하는지 확인한다.
```

Jetson에서:

```bash
ls -l /dev/ttyUSB*
```

LED가 켜지고 `/dev/ttyUSB0`가 다시 나타나면 정상이다.

현재 환경에서는 **Converter의 6핀 쪽을 뺐다가 다시 꽂으면 정상화되는 것을 확인했다.**

---

# 4. Cube Orange 부팅 후 확인

Cube를 켜고 QGroundControl에 연결한다.

QGC의 **MAVLink Console**에서:

```bash
uxrce_dds_client status
```

Jetson Agent를 아직 실행하지 않은 상태에서는 일반적으로:

```text
Running, disconnected
```

이면 된다.

만약:

```text
not running
```

이면 Cube를 재부팅하는 것이 가장 간단하다.

QGC MAVLink Console:

```bash
reboot
```

부팅 완료 후 다시:

```bash
uxrce_dds_client status
```

를 확인한다.

---

# 5. Jetson 터미널 1
# MicroXRCEAgent 실행

Jetson에서 **터미널 1**을 연다.

먼저 USB-UART 확인:

```bash
ls -l /dev/ttyUSB*
```

현재 `/dev/ttyUSB0`인 경우 다음을 실행한다.

```bash
단축어 : exportros
source /opt/ros/humble/setup.bash
source ~/px4_ros_uxrce_dds_ws/install/local_setup.bash

unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset CYCLONEDDS_URI

export ROS_DOMAIN_ID=0
```

그다음 MicroXRCEAgent 실행:
```bash
단축어: uxrce
MicroXRCEAgent serial -D /dev/ttyUSB0 -b 921600 -v 6
```

이 터미널은 **계속 켜둔다.**

권한 오류가 발생하면:

```bash
sudo $(which MicroXRCEAgent) serial -D /dev/ttyUSB0 -b 921600 -v 6
```

를 사용한다.

---

# 6. Cube ↔ Jetson XRCE 연결 확인

MicroXRCEAgent를 실행한 상태에서 QGC MAVLink Console:

```bash
uxrce_dds_client status
```

정상 상태:

```text
Running, connected
```

여기까지 확인되면:

```text
Cube
 ↕ UART
UART-to-USB
 ↕
Jetson MicroXRCEAgent
```

구간은 정상이다.

---

# 7. Jetson px4_msgs 버전

현재 Cube의 PX4 버전은:

```text
PX4 v1.17.0
```

이므로 Jetson의 `px4_msgs`도 `v1.17.0`으로 맞췄다.

현재 `VehicleStatus`의 message version은:

```text
MESSAGE_VERSION = 1
```

이다.

Jetson **터미널 2**에서 확인:

```bash
source /opt/ros/humble/setup.bash
source ~/Aircraft_ws/install/setup.bash

python3 - <<'PY'
from px4_msgs.msg import VehicleStatus
print("MESSAGE_VERSION =", VehicleStatus.MESSAGE_VERSION)
PY
```

정상:

```text
MESSAGE_VERSION = 1
```

---

# 8. Jetson 터미널 2
# ROS2 환경 설정

MicroXRCEAgent가 실행되는 터미널과 별도로 **Jetson 터미널 2**를 연다.

```bash
source /opt/ros/humble/setup.bash
source ~/Aircraft_ws/install/setup.bash

unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset CYCLONEDDS_URI

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```

이전에 Raspberry Pi + Fast DDS Discovery Server를 사용했기 때문에,
직접 연결에서는 다음 환경변수를 반드시 제거한다.

```bash
unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset CYCLONEDDS_URI
```

ROS2 daemon도 초기화한다.

```bash
ros2 daemon stop
sleep 2
```

---

# 9. PX4 토픽 확인

Jetson 터미널 2:

```bash
ros2 topic list | grep /fmu
```

`VehicleStatus`만 확인:

```bash
ros2 topic list | grep vehicle_status
```

현재 사용하는 토픽:

```text
/fmu/out/vehicle_status_v1
```

Publisher 상세 확인:

```bash
ros2 topic info -v /fmu/out/vehicle_status_v1
```

정상 예:

```text
Type: px4_msgs/msg/VehicleStatus
Publisher count: 1

Reliability: BEST_EFFORT
Durability: TRANSIENT_LOCAL
```

---

# 10. FC → Jetson 실제 데이터 수신 확인

Jetson 터미널 2:

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 \
  --qos-reliability best_effort \
  --qos-durability transient_local
```

정상이면 `VehicleStatus` 데이터가 계속 출력된다.

이 단계까지 성공하면:

```text
Cube
 ↓
VehicleStatus
 ↓
Micro XRCE-DDS
 ↓
Jetson
 ↓
ROS2
```

실제 데이터 수신까지 정상이다.

---

# 11. Jetson → FC 통신 확인

테스트 Python 노드에서 다음 토픽을 발행하여 역방향 통신도 확인했다.

```text
/fmu/in/onboard_computer_status
```

Jetson에서 테스트 노드를 실행한 상태에서
QGC MAVLink Console:

```bash
listener onboard_computer_status 5
```

`timestamp`, `uptime` 등이 계속 갱신되면 정상이다.

확인된 결과:

```text
Jetson
 ↓
ROS2 publish
 ↓
MicroXRCEAgent
 ↓
UART
 ↓
Cube
 ↓
uORB onboard_computer_status
```

역방향 통신도 성공했다.

---

# 12. 재부팅 후 실제 실행 순서

## Step 1. Cube와 Converter 연결

```text
Cube TELEM2
    ↓
UART-to-USB Converter
    ↓
Jetson USB
```

Converter LED가 들어오는지 확인한다.

LED가 꺼져 있으면:

```text
Converter의 6핀 커넥터를 뺐다가 다시 꽂는다.
```

그다음 Jetson에서:

```bash
ls -l /dev/ttyUSB*
```

확인.

---

## Step 2. Cube 상태 확인

QGC MAVLink Console:

```bash
uxrce_dds_client status
```

Agent 실행 전:

```text
Running, disconnected
```

이면 정상.

---

## Step 3. Jetson 터미널 1 - MicroXRCEAgent

```bash
source /opt/ros/humble/setup.bash
source ~/px4_ros_uxrce_dds_ws/install/local_setup.bash

unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset CYCLONEDDS_URI

export ROS_DOMAIN_ID=0

MicroXRCEAgent serial -D /dev/ttyUSB0 -b 921600 -v 6
```

이 터미널은 계속 실행한다.

---

## Step 4. QGC에서 연결 확인

```bash
uxrce_dds_client status
```

정상:

```text
Running, connected
```

---

## Step 5. Jetson 터미널 2 - ROS2

```bash
단축어:aircraft_ros
source /opt/ros/humble/setup.bash
source ~/Aircraft_ws/install/setup.bash

unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset CYCLONEDDS_URI

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

ros2 daemon stop
sleep 2
```

---

## Step 6. 토픽 확인

```bash
ros2 topic list | grep /fmu
```

```bash
ros2 topic list | grep vehicle_status
```

---

## Step 7. 실제 데이터 확인

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 \
  --qos-reliability best_effort \
  --qos-durability transient_local
```

데이터가 계속 출력되면 전체 연결 성공이다.

---

# 13. 현재 최종 터미널 구성

```text
[QGC MAVLink Console]

uxrce_dds_client status

역할:
Cube의 XRCE client 상태 확인
```

```text
[Jetson 터미널 1]

MicroXRCEAgent

역할:
Cube UART ↔ ROS2 DDS bridge
```

```text
[Jetson 터미널 2]

ROS2 topic list / echo
fixedwing_autonomy
테스트 Python node

역할:
실제 ROS2 application 실행
```

---


