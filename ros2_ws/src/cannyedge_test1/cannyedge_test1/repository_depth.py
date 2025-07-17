#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cannyedge_test1_interfaces.msg import HoleInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters


class CannyEdgeDetector(Node):
    def __init__(self):
        super().__init__("canny_edge_detector")
        self.get_logger().info(
            "Starting Canny Edge Detector (Simulating D435i, Output: Camera Optical Frame)."
        )
        self.cv_bridge = CvBridge()

        # 카메라 내부 파라미터
        self.fx = self.fy = self.cx = self.cy = None
        # Temporal filter state
        self.prev_depth = None

        # 🔴시뮬레이션 vs real world 토픽 설정

        # 시뮬레이션 D435i 카메라 사용 시 아래 주석 해제
        # color_image_topic = "/d435i/color/image_raw"
        # camera_info_topic = "/d435i/depth/camera_info"
        # depth_image_topic = "/d435i/depth/image_raw"

        # real world D435i 카메라 사용 시 아래 주석 해제
        color_image_topic = "/camera/camera/color/image_raw"
        camera_info_topic = "/camera/camera/depth/camera_info"
        depth_image_topic = "/camera/camera/depth/image_rect_raw"

        # CameraInfo 구독
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            rclpy.qos.QoSProfile(
                depth=10,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
            )
        )
        # Color/Depth 이미지 동기화 구독
        color_sub = message_filters.Subscriber(self, Image, color_image_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_image_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.2
        )
        # 콜백 등록
        self.ts.registerCallback(self.sync_callback)

        self.image_pub = self.create_publisher(Image, "/image_edge", 10)
        self.hole_pub = self.create_publisher(HoleInfo, "/hole_info", 10)
        self.get_logger().info("Canny Edge Detector node initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        # 카메라 내부 파라미터 획득 (수정됨)
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]

    def pixel_to_3d(self, u, v, depth):
        # 3D 포인트 계산 (에러 처리 강화)
        if None in (self.fx, self.fy, self.cx, self.cy):
            return None
        if depth <= 0 or not np.isfinite(depth):
            return None

        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def sync_callback(self, color_msg: Image, depth_msg: Image):
        # 초기 HoleInfo 메시지
        hole_info_msg = HoleInfo(
            find_hole=False,
            passable=False,
            center=Point(),
            width=0.0,
            height=0.0
        )

        # 카메라 파라미터 준비 확인
        if None in (self.fx, self.fy, self.cx, self.cy):
            self.hole_pub.publish(hole_info_msg)
            return

        # 컬러 이미지 변환
        try:
            cv_color = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")
            self.hole_pub.publish(hole_info_msg)
            return

        # 깊이 이미지 변환
        try:
            depth_cv = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            self.hole_pub.publish(hole_info_msg)
            return

        # 1) Temporal smoothing (EMA 필터) - 활성화
        alpha = 0.2  # EMA 계수 (0.0~1.0, 클수록 이전 프레임 영향 더 큼)
        if self.prev_depth is None:
            self.prev_depth = depth_cv.copy()
        else:
            depth_cv = alpha * depth_cv + (1 - alpha) * self.prev_depth
            self.prev_depth = depth_cv.copy()

        # 2) 🔴Invalid depth 값 제거(clip distance랑 유사)
        depth_cv = depth_cv.astype(np.float32)
        depth_cv[depth_cv <= 0] = np.nan
        depth_cv[depth_cv > 4000] = np.nan  # 몇 m 이상 제거

        # 3) Median 필터 추가 (salt-and-pepper 노이즈 제거)
        depth_cv = cv2.medianBlur(depth_cv.astype(np.float32), 3)

        # 4) Bilateral 필터 (sigmaColor 50~100)
        depth_cv = cv2.bilateralFilter(depth_cv, d=3, sigmaColor=50, sigmaSpace=3)

        # 5) 컬러맵 시각화 (동적 정규화)
        valid = np.isfinite(depth_cv) & (depth_cv > 0)
        if np.any(valid):
            min_d, max_d = float(np.min(depth_cv[valid])), float(np.max(depth_cv[valid]))
            if max_d > min_d:
                norm = ((depth_cv - min_d) / (max_d - min_d) * 255).clip(0, 255).astype(np.uint8)
            else:
                norm = np.zeros_like(depth_cv, dtype=np.uint8)
        else:
            norm = np.zeros_like(depth_cv, dtype=np.uint8)
        result_image = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)

        # 화면 중앙 및 가이드라인
        h, w = result_image.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(result_image, (0, cy), (w, cy), (255, 0, 0), 1)
        cv2.line(result_image, (cx, 0), (cx, h), (255, 0, 0), 1)
        tol = 23
        cv2.rectangle(result_image, (cx-tol, cy-tol), (cx+tol, cy+tol), (255, 0, 0), 1)

        # 6) 컬러맵 기반 컨투어 검출
        # gray_cm = cv2.cvtColor(result_image, cv2.COLOR_BGR2GRAY)
        # _, cm_mask = cv2.threshold(gray_cm, 200, 255, cv2.THRESH_BINARY)

        hsv = cv2.cvtColor(result_image, cv2.COLOR_BGR2HSV)
        cm_mask = cv2.inRange(hsv,
                    np.array([15, 100, 100]),    # Hue 약 15°, Sat 100↑, Val 100↑
                    np.array([35, 255, 255]))    # Hue 약 35°


        # Morphological 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        cm_mask = cv2.morphologyEx(cm_mask, cv2.MORPH_OPEN, kernel)
        cm_mask = cv2.morphologyEx(cm_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(cm_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 7) 구멍 후보 필터링 및 HoleInfo 작성 (개선됨)
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
            area = cv2.contourArea(approx)
            x,y,w_box,h_box = cv2.boundingRect(cnt)
            ar = float(w_box) / float(h_box)
            if 0.5 <= ar <= 2.0 and area > 1000 and len(approx) == 4:
                x, y, w_box, h_box = cv2.boundingRect(approx)
                if h_box == 0:
                    continue
                ar = float(w_box) / h_box
                if not (0.5 <= ar <= 2.0):
                    continue

                # 중심점
                M = cv2.moments(approx)
                if M["m00"] == 0:
                    continue
                px = int(M["m10"] / M["m00"])
                py = int(M["m01"] / M["m00"])

                # 바운딩 박스 시각화
                cv2.drawContours(result_image, [approx], -1, (0, 255, 0), 2)
                # 무게 중심 점 표시
                cv2.circle(result_image, (px, py), 5, (0, 0, 255), -1)

                # 🔴 임시 depth=1.0m
                depth_est = 1.0

                coords = self.pixel_to_3d(px, py, depth_est)
                if coords is None:
                    continue
                Xc, Yc, Zc = coords
                # 실제 크기
                real_w = (w_box / self.fx) * depth_est
                real_h = (h_box / self.fy) * depth_est
                passable = (real_w >= 0.5 and real_h >= 0.5)

                # HoleInfo 덮어쓰기
                hole_info_msg.find_hole = True
                hole_info_msg.passable = bool(passable)
                hole_info_msg.center = Point(x=Xc, y=Yc, z=Zc)
                hole_info_msg.width = real_w
                hole_info_msg.height = real_h
                break

        # 8) 퍼블리시
        self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(result_image, "bgr8"))
        self.hole_pub.publish(hole_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
