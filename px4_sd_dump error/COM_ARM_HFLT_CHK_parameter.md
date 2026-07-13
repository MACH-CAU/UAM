# COM_ARM_HFLT_CHK Parameter

먼저 `px4_sd_dump.md` 파일의 내용을 확인하고, 해당 문서에 제시된 방법으로 해결해보기.

문제가 해결되지 않는 경우에는 QGroundControl의 기체 파라미터 설정에서 다음 값 변경.

```text
COM_ARM_HFLT_CHK = Disabled (0)
```

COM_ARM_HFLT_CHK

PX4의 COM_ARM_HFLT_CHK 파라미터는 기체를 arming 하기 전에 SD 카드에 Hardfault 또는 Watchdog 오류 기록이 존재하는지 확인하는 기능

해당 파라미터를 Disabled (0)로 설정하면 SD 카드에 오류 기록이 존재하더라도 arming 가능

주의사항

COM_ARM_HFLT_CHK를 비활성화하는 것은 SD 카드 내부의 오류 원인을 해결하는 것이 아니라, 해당 검사를 건너뛰고 강제로 arming이 가능하도록 하는 임시방편임.

따라서 실제 오류 원인을 해결하지 않은 상태에서 비행할 경우 예기치 않은 시스템 오류나 비행 이상으로 이어질 가능성이 있으므로, 원인을 확인하지 못한 상태에서의 실제 비행은 권장하지 않음.
