# Jetson Nano에서 yolo camera 실행
## 1. intel camera 연결 
jetson 에 케이블로 연결 후  
터미널에서 

``` 
ssh -Y vtol@와이파이-IP
 ``` 

접속
## 2. 이미지 인식 실행
jetson 터미널에서

```
cd ~/gate_detection/
python3 ~/gate_detection/scripts/realsense_trt_live.py
```
