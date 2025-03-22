

# Essential Workspace

### 무엇?

최종적을 필요한 패키지들만 모아둔 곳

여러 검증이 완료되었을 때,
심볼릭 링크로 생성하여 따로 관리


### 사용법


그냥 각각 폴더에서 `colcon build` 하면 됨
다만, MUST!!!!! 환경변수 업데이트 해줘야 함

```shell
cd UAM/Essential_Workspace/

colcon build

source install/setup.bash

```


