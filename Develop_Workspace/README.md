

# Essential Workspace

### 무엇?

개발하는 과정에서 사용했던 패키지들만 모아둔 곳

백업느낌이면서, 실제적으로 코드를 작성하는 공간임
essential이랑 심볼릭 링크되어 있기 때문에 영향 미치는 것을 염두해야 함


### 사용법


그냥 각각 폴더에서 `colcon build` 하면 됨
다만, MUST!!!!! 환경변수 업데이트 해줘야 함

```shell
cd UAM/Develop_Workspace/

colcon build

source install/setup.bash

```


