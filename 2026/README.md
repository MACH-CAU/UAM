# 2026 MACH VTOL

## PX4 SD Card Dump
#### 이 과정은 QGC Analyze Tool에 있는 MAVLink Console에서 진행!
### 문제

- QGC 연결/해제 또는 재부팅 시 `SD card Dump` 발생
- SD카드에 `fault_*.log` 반복 생성
- Arm 전후 통신 끊김 또는 FC 재부팅 발생
- EKF, IMU, Compass, GPS warning 발생

---

### Fault Log 확인

SD카드의 fault log를 확인한 결과, SD카드 자체 문제가 아니라 PX4 내부 HardFault로 확인됨.

```txt
Type: Hard Fault
running task: mavlink_if2
cfsr: 0x01000000
hfsr: 0x40000000
```

따라서 원인은 SD카드가 아니라 `mavlink_if2` task의 HardFault로 판단함.

---

### 원인 확인

QGC MAVLink Console에서 MAVLink 상태를 확인함.

```bash
mavlink status
```

초기 상태에서 다음과 같은 불필요한 MAVLink 링크가 확인됨.

```txt
/dev/ttyS4 @57600
mode: Minimal
```

정상적으로 필요한 링크는 USB 또는 TELEM1이었으나, `/dev/ttyS4` 링크가 추가로 실행 중이었음.

---

### 조치 내용

불필요한 `/dev/ttyS4` MAVLink 인스턴스를 종료함.

```bash
mavlink stop -d /dev/ttyS4
```

이후 다시 확인함.

```bash
mavlink status
```

조치 후 `/dev/ttyS4 @57600 Minimal` 링크가 사라짐.

기존 fault log는 삭제함.

```bash
rm /fs/microsd/fault_*.log
ls /fs/microsd
```

---

### 결과

- USB 단독 연결 시 `fault_*.log` 재생성 안 됨
- 텔레메트리 단독 연결 시 QGC 정상 연결
- QGC 연결/해제 후 SD card dump 재발하지 않음
- 센서 캘리브레이션 및 모터 테스트 정상 수행
- GPS warning 외 추가 health warning 없음 (GPS warning은 실내여서 세팅 안 한 상태였음.)

---

### 최종

`SD card Dump`는 SD카드 고장이 아니라 PX4 HardFault로 인해 생성된 crash dump였음.

유력 원인은 불필요하게 실행 중이던 `/dev/ttyS4 @57600 Minimal` MAVLink 인스턴스이며, 해당 링크를 종료한 후 문제가 재현되지 않음. 확실한 지는 실제로 비행 테스트를 외부에서 해봐야 할 것 같음.

---

### 비행 전 확인 사항

재부팅 후 `/dev/ttyS4` 링크가 다시 살아날 수 있으므로 비행 전 확인 필요.

```bash
mavlink status
```

만약 아래 링크가 보이면:

```txt
/dev/ttyS4 @57600
mode: Minimal
```

다시 종료함.

```bash
mavlink stop -d /dev/ttyS4
```
