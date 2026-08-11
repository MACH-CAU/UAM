# Cube Orange → Raspberry Pi → Jetson ROS2 실행 절차

> 현재 상태  
> - Cube Orange ↔ Raspberry Pi Micro XRCE-DDS 연결: **성공**
> - Raspberry Pi ↔ Jetson 양방향 ping: **성공**
> - Jetson에서 `/fmu/...` 토픽 목록 discovery: **성공**
> - Jetson에서 `/fmu/out/vehicle_status_v1` 실제 message `echo`: **아직 안 나옴**
>
> 따라서 현재는 **DDS 토픽 discovery는 되지만 실제 payload 수신은 아직 미확인** 상태다.

---

## 0. 현재 주소 / 장치

```text
Raspberry Pi IP : 10.210.56.90
Jetson IP       : 10.210.156.37

Cube ↔ Pi UART  : /dev/serial0
UART baud       : 921600

Fast DDS Discovery Server:
10.210.56.90:11811
```

---

# 1. 전체 실행 순서

재부팅 후에는 아래 순서대로 실행한다.

```text
1. Cube Orange 전원 ON
2. Raspberry Pi 전원 ON
3. Jetson 전원 ON

4. Pi 터미널 1
   → Fast DDS Discovery Server 실행

5. Pi 터미널 2
   → MicroXRCEAgent 실행

6. QGC MAVLink Console
   → Cube XRCE 연결 상태 확인

7. Jetson 터미널
   → ROS2/Fast DDS 환경 설정
   → PX4 토픽 확인
```

---

# 2. Raspberry Pi 터미널 1
# Fast DDS Discovery Server

Pi에서 **새 터미널 1**을 연다.

```bash
source /opt/ros/humble/setup.bash

fastdds discovery -i 0 -l 10.210.56.90 -p 11811
```

이 터미널은 **종료하지 않는다.**

정상 실행 여부를 별도 터미널에서 확인하려면:

```bash
ss -lunp | grep 11811
```

---

# 3. Raspberry Pi 터미널 2
# MicroXRCEAgent

Pi에서 **새 터미널 2**를 연다.

먼저 UART 확인:

```bash
ls -l /dev/serial0
```

현재 정상 장치는:

```text
/dev/serial0 -> ttyAMA0
```

Agent 실행:

```bash
source /opt/ros/humble/setup.bash

export ROS_DISCOVERY_SERVER=10.210.56.90:11811
export ROS_DOMAIN_ID=0

sudo -E MicroXRCEAgent serial -D /dev/serial0 -b 921600 -v 6
```

이 터미널도 **종료하지 않는다.**

---

# 4. QGroundControl MAVLink Console
# Cube ↔ Pi XRCE 연결 확인

QGC에서:

```text
Vehicle Setup 또는 Analyze Tools
→ MAVLink Console
```

다음 명령 실행:

```bash
uxrce_dds_client status
```

정상 목표:

```text
Running, connected
```

`Running, disconnected`이면 Jetson 문제가 아니라
먼저 Cube ↔ Pi MicroXRCEAgent 구간을 확인한다.

---

# 5. Pi ↔ Jetson 네트워크 확인

## Jetson → Pi

Jetson에서:

```bash
ping -c 4 10.210.56.90
```

## Pi → Jetson

Pi에서:

```bash
ping -c 4 10.210.156.37
```

둘 다 `0% packet loss`인지 확인한다.

---

# 6. Jetson 터미널
# ROS2 + Fast DDS 설정

Jetson에서 **새 터미널**을 열 때마다 아래 설정을 먼저 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/Aircraft_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER=10.210.56.90:11811
export ROS_SUPER_CLIENT=TRUE
```

현재 설정 확인:

```bash
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
echo $ROS_DISCOVERY_SERVER
echo $ROS_SUPER_CLIENT
```

기대값:

```text
rmw_fastrtps_cpp
0
10.210.56.90:11811
TRUE
```

ROS2 daemon 초기화:

```bash
ros2 daemon stop
sleep 2
```

---

# 7. Jetson에서 PX4 토픽 목록 확인

```bash
ros2 topic list | grep /fmu
```

vehicle_status만 확인:

```bash
ros2 topic list | grep vehicle_status
```

현재 확인한 토픽 이름이 다음이라면:

```text
/fmu/out/vehicle_status_v1
```

이후 명령에서는 정확히 이 이름을 사용한다.

> 터미널 명령 앞에 `$`를 직접 입력하지 않는다.  
> `_` 앞에 `\`도 넣지 않는다.

잘못된 예:

```text
$ ros2 topic echo /fmu/out/vehicle\_status\_v1
```

올바른 예:

```bash
ros2 topic echo /fmu/out/vehicle_status_v1
```

---

# 8. Jetson에서 실제 PX4 데이터 확인

먼저 기본 echo:

```bash
ros2 topic echo /fmu/out/vehicle_status_v1
```

PX4 QoS를 명시해서 확인:

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 \
  --qos-reliability best_effort \
  --qos-durability transient_local
```

현재는 위 명령을 실행해도 message가 나오지 않는 상태다.

---

# 9. 현재 문제 디버깅
# 토픽 목록은 보이는데 echo가 안 나올 때

## 9-1. Jetson에서 publisher endpoint 확인

```bash
ros2 topic info -v /fmu/out/vehicle_status_v1
```

확인할 것:

```text
Publisher count
Node name
Reliability
Durability
```

---

## 9-2. 토픽 타입 확인

```bash
ros2 topic type /fmu/out/vehicle_status_v1
```

예상 형태:

```text
px4_msgs/msg/VehicleStatus
```

또는 PX4 버전에 따라 versioned message type이 표시될 수 있다.

---

## 9-3. Aircraft_ws에 px4_msgs가 실제로 잡히는지 확인

Jetson에서:

```bash
source /opt/ros/humble/setup.bash
source ~/Aircraft_ws/install/setup.bash

ros2 pkg list | grep px4_msgs
```

출력이 있어야 한다.

추가 확인:

```bash
ros2 interface list | grep VehicleStatus
```

---

## 9-4. Jetson DDS 환경 다시 확인

```bash
env | grep -E 'RMW_IMPLEMENTATION|ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT'
```

기대값:

```text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=0
ROS_DISCOVERY_SERVER=10.210.56.90:11811
ROS_SUPER_CLIENT=TRUE
```

---

## 9-5. Pi의 Agent 환경 확인

MicroXRCEAgent를 실행한 Pi 터미널과 같은 환경에서:

```bash
echo $ROS_DISCOVERY_SERVER
echo $ROS_DOMAIN_ID
```

기대값:

```text
10.210.56.90:11811
0
```

Agent는 반드시 다음처럼 `sudo -E`로 실행한다.

```bash
sudo -E MicroXRCEAgent serial -D /dev/serial0 -b 921600 -v 6
```

---

## 9-6. Discovery Server가 실제로 떠 있는지 Pi에서 확인

Pi의 별도 터미널:

```bash
ss -lunp | grep 11811
```

안 나오면 Discovery Server를 다시 실행:

```bash
source /opt/ros/humble/setup.bash
fastdds discovery -i 0 -l 10.210.56.90 -p 11811
```

---

# 10. DDS 패킷 실제 송수신 확인
# echo가 계속 안 나올 때

## Jetson에서 Pi 관련 UDP 패킷 확인

Jetson 새 터미널:

```bash
sudo tcpdump -ni any host 10.210.56.90 and udp
```

## Pi에서 Jetson 관련 UDP 패킷 확인

Pi 새 터미널:

```bash
sudo tcpdump -ni any host 10.210.156.37 and udp
```

통신 시험 중 양쪽에서 패킷이 움직이는지 확인한다.

종료:

```text
Ctrl+C
```

---

# 11. Jetson 자체 ROS2 publish/subscribe 테스트

PX4와 무관하게 Jetson ROS2가 정상인지 확인할 때 사용한다.

## Jetson 터미널 1

```bash
source /opt/ros/humble/setup.bash

python3 - <<'PY'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('test_publisher')
pub = node.create_publisher(String, '/test_topic', 10)

i = 0
try:
    while rclpy.ok():
        msg = String()
        msg.data = f'Jetson test {i}'
        pub.publish(msg)
        print(f'Published: {msg.data}')
        i += 1
        rclpy.spin_once(node, timeout_sec=1.0)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

## Jetson 터미널 2

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /test_topic
```

이 테스트는 현재 정상 동작 확인 완료.

---

# 12. 현재 터미널 구성

실제 운용 중에는 최소 아래 터미널을 유지한다.

```text
[Raspberry Pi 터미널 1]
Fast DDS Discovery Server

[Raspberry Pi 터미널 2]
MicroXRCEAgent

[Jetson 터미널 1]
ROS2 / PX4 topic 확인 또는 fixedwing_autonomy 실행

[QGC MAVLink Console]
uxrce_dds_client status 확인
```

필요한 경우 디버깅용으로 추가:

```text
[Jetson 터미널 2]
tcpdump / ros2 topic info / echo

[Pi 터미널 3]
tcpdump / ss / ping
```

---

# 13. 재부팅 후 복붙용 명령

## Raspberry Pi 터미널 1

```bash
source /opt/ros/humble/setup.bash
fastdds discovery -i 0 -l 10.210.56.90 -p 11811
```

## Raspberry Pi 터미널 2

```bash
source /opt/ros/humble/setup.bash
export ROS_DISCOVERY_SERVER=10.210.56.90:11811
export ROS_DOMAIN_ID=0
sudo -E MicroXRCEAgent serial -D /dev/serial0 -b 921600 -v 6
```

## Jetson 터미널

```bash
source /opt/ros/humble/setup.bash
source ~/Aircraft_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER=10.210.56.90:11811
export ROS_SUPER_CLIENT=TRUE

ros2 daemon stop
sleep 2

ros2 topic list | grep /fmu
```

## Jetson vehicle_status 확인

```bash
ros2 topic list | grep vehicle_status
```

```bash
ros2 topic info -v /fmu/out/vehicle_status_v1
```

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 \
  --qos-reliability best_effort \
  --qos-durability transient_local
```

## QGC MAVLink Console

```bash
uxrce_dds_client status
```

---

# 14. 현재 성공 / 미확인 상태

```text
[성공]
Cube Orange ↔ Raspberry Pi UART
Cube uxrce_dds_client ↔ Pi MicroXRCEAgent
Pi ↔ Jetson 양방향 ping
Pi Fast DDS Discovery Server
Jetson에서 /fmu/... topic discovery
Jetson 자체 ROS2 Python pub/sub

[아직 미확인]
Pi DDS → Jetson 실제 PX4 payload 수신
Jetson → Cube ROS2 publish
fixedwing_autonomy 실제 연결
```
