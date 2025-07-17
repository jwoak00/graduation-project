# Canny Edge Detector (ROS 2 Humble)

Intel RealSense **D435i** RGB-D 카메라로  
드론 시야의 **오른쪽 ¾ 영역(ROI)** 에서 “구멍”을 탐지하고  
`HoleInfo` 커스텀 메시지에 **3-D 센터 좌표 · 크기 · 통과 가능 여부**를 퍼블리시하는 노드입니다.

<p align="center">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu" />
  <img src="https://img.shields.io/badge/ROS2-Humble%20Hawksbill-22314E?logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python" />
</p>

---

##  패키지 구조
```text
cannyedge_test1/
├── cannyedge_detector.py       # 본 노드
├── msg/HoleInfo.msg            # 구멍 정보 메시지
├── launch/                     # 예시 런치파일
└── package.xml  /  setup.py
```

##  주요 기능
| 기능 | 설명 |
|------|------|
| **ROI 자르기** | 컬러·깊이 프레임의 **오른쪽 75 %** 만 분석 → 연산 ¼ 절감 |
| **심도 노이즈 필터링** | EMA + `medianBlur` + `bilateralFilter` 로 실시간 심도 안정화 |
| **Canny-HSV 혼합 검출** | INFERNO 컬러맵 → HSV 룩업으로 사각 컨투어를 빠르게 검출 |
| **3-D 치수 추정** | 카메라 내부 파라미터로 실거리 폭·높이 산출 |
| **통과 가능 판정** | 폭·높이 ≥ 0.5 m 이면 `passable = true` |
| **디버그 이미지** | `/image_edge` 토픽에 컬러맵·컨투어 시각화 송출 |

---

##  의존성
```bash
sudo apt install ros-humble-rmw-fastrtps-cpp \
                 ros-humble-image-transport \
                 ros-humble-cv-bridge \
                 ros-humble-message-filters \
                 librealsense2-dkms librealsense2-utils
pip install opencv-python numpy
```
> **Tip** : RealSense 드라이버 노드(`realsense2_camera`)를 먼저 실행하세요.

---

##  빌드 · 설치
```bash
cd ~/ros2_ws/src
git clone <repo-url>
cd ..
rosdep install --from-paths src -r -y       # 의존성 자동 설치
colcon build --packages-select cannyedge_test1
source install/setup.bash
```

---

## ▶ 실행 예시

### 1. 실물 D435i
```bash
# RealSense 드라이버
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=848x480x30  color_module.profile:=848x480x30

# Canny Edge Detector
ros2 run cannyedge_test1 canny_edge_detector
```

### 2. Gazebo 시뮬레이션 (OAK-D Lite 출력 → D435i 토픽으로 리맵)
```bash
ros2 run cannyedge_test1 canny_edge_detector \
  --ros-args \
  -r /camera/camera/color/image_raw:=/d435i/color/image_raw \
  -r /camera/camera/depth/image_rect_raw:=/d435i/depth/image_raw \
  -r /camera/camera/depth/camera_info:=/d435i/depth/camera_info
```
<img width="1920" height="1200" alt="image" src="https://github.com/user-attachments/assets/11b9f21e-c5cf-49cb-9d77-fb725cfa36a3" />

---

##  토픽 I/O

| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| **Sub** | `/camera/camera/color/image_raw` | `sensor_msgs/Image` | BGR 컬러 |
| **Sub** | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 16-bit Depth (mm) |
| **Sub** | `/camera/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 카메라 내참수 |
| **Pub** | `/image_edge` | `sensor_msgs/Image` | 디버그 시각화 |
| **Pub** | `/hole_info` | `cannyedge_test1_interfaces/HoleInfo` | 구멍 통과 정보 |

### `HoleInfo.msg`
```text
bool   find_hole          # 구멍 검출 여부
bool   passable           # 통과 가능? (폭·높이 ≥ 임계값)
geometry_msgs/Point center  # 카메라 기준 3-D 좌표 (m)
float32 width             # 구멍 폭  (m)
float32 height            # 구멍 높이 (m)
```

---

##  노드 파라미터
| 이름 | 기본값 | 설명 |
|------|--------|------|
| `~loop_hz`             | `20`  | 메인 루프 주기 |
| `~min_passable_width`  | `0.5` | 폭 임계값 (m) |
| `~min_passable_height` | `0.5` | 높이 임계값 (m) |

---

##  rqt2 빠른 시각화
```bash
rqt2 # /image_edge 선택
```

---

<img width="1920" height="1200" alt="image" src="https://github.com/user-attachments/assets/a64627d4-a82d-442d-b1c2-07062e021dc1" />


