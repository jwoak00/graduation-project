# Canny Edge Detector (ROS 2 Humble)

Intel RealSense **D435i** RGB-D 카메라 컬러 + 깊이 프레임을 동기화해  
Canny Edge 기반으로 **사각 구멍**을 탐지하고,  
`HoleInfo`(커스텀 메시지)로 **3-D 중심 좌표 · 실제 크기 · 통과 가능 여부**를 퍼블리시하는 노드입니다.  

<p align="center">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu" />
  <img src="https://img.shields.io/badge/ROS2-Humble%20Hawksbill-22314E?logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python" />
</p>

---

## 📂 패키지 구조
```text
cannyedge_test1/
├── cannyedge_detector.py       # 본 노드 (아래 코드)
├── msg/HoleInfo.msg            # 구멍 정보 메시지 정의
├── launch/                     # 예시 런치파일
└── package.xml  /  setup.py
```

---

## 🚀 주요 기능
| 기능 | 설명 |
|------|------|
| **컬러 + 깊이 동기화** | `message_filters` ATS 사용(슬롭 0.2 s) |
| **Canny Edge 검출** | `threshold1/2` 파라미터를 런타임에 동적으로 조정 |
| **모폴로지 Close** | 엣지 틈새를 메워 사각형 컨투어 검출 안정 |
| **3-D 변환** | 카메라 내참수(`fx`,`fy`,`cx`,`cy`) → 광학 좌표계 산출 |
| **통과 가능 판정** | 폭·높이 ≥ 0.5 m → `passable=true` |
| **실시간 파라미터 변경** | `ros2 param set /canny_edge_detector threshold1 120` 와 같이 즉시 반영 |
| **디버그 이미지** | `/image_edge` 토픽에 시각화(BBox·중심점·가이드라인) 퍼블리시 |

> ⚠️ 현재 깊이값은 **임시 상수 2 m** 로 추정합니다. RealSense 실제 깊이를 사용하려면 `depth_est = depth_msg` 데이터로 교체해 주세요.

<img width="1920" height="1200" alt="image" src="https://github.com/user-attachments/assets/5ba4b9e5-17da-4261-9dc2-74077af40e42" />

---

## 의존성
```bash
sudo apt install ros-humble-rmw-fastrtps-cpp \
                 ros-humble-image-transport \
                 ros-humble-cv-bridge \
                 ros-humble-message-filters \
                 librealsense2-dkms librealsense2-utils
pip install opencv-python numpy
```
> **Tip** : RealSense 드라이버 노드(`realsense2_camera`)를 먼저 실행해야 컬러·깊이 · CameraInfo 토픽이 발행됩니다.

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

##  실행 예시
```bash
# 1) RealSense 드라이버
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=848x480x30  color_module.profile:=848x480x30

# 2) Canny Edge Detector 노드
ros2 run cannyedge_test1 canny_edge_detector
```

런타임 조정 예시:
```bash
ros2 param set /canny_edge_detector threshold1 80
ros2 param set /canny_edge_detector threshold2 180
```

---

##  토픽 I/O

| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| **Sub** | `/camera/camera/color/image_raw`      | `sensor_msgs/Image`   | BGR 컬러 |
| **Sub** | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image`   | 16-bit Depth (mm) ※현재 깊이값 미사용 |
| **Sub** | `/camera/camera/color/camera_info`    | `sensor_msgs/CameraInfo` | 카메라 내참수 |
| **Pub** | `/image_edge`                         | `sensor_msgs/Image`   | 디버그 시각화 |
| **Pub** | `/hole_info`                          | `cannyedge_test1_interfaces/HoleInfo` | 구멍 정보 |

### `HoleInfo.msg`
```text
bool   find_hole          # 구멍 검출 여부
bool   passable           # 통과 가능? (폭·높이 ≥ 0.5 m)
geometry_msgs/Point center  # 카메라 기준 3-D 좌표 (m)
float32 width             # 구멍 폭  (m)
float32 height            # 구멍 높이 (m)
```

---

##  파라미터

| 이름 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `threshold1` | int | `100` | Canny Edge lower 임계값 |
| `threshold2` | int | `200` | Canny Edge upper 임계값 |

모두 런타임에서 `ros2 param set` 으로 즉시 변경 가능.

---

##  rqt2 빠른 시각화
```bash
rqt2 # /image_edge 토픽 선택
```
<img width="1920" height="1200" alt="image" src="https://github.com/user-attachments/assets/4735f03f-30db-4f99-b1b2-520d7ee28595" />

---

