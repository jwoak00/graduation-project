#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.qos
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cannyedge_test1_interfaces.msg import HoleInfo  # 커스텀 메시지
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult


class CannyEdgeDetector(Node):
    def __init__(self):
        super().__init__('canny_edge_detector')
        self.get_logger().info(
            "Starting Canny Edge Detector (Simulating D435i, Output: Camera Optical Frame)."
        )
        self.cv_bridge = CvBridge()

        # 카메라 내부 파라미터 초기화
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.last_camera_info_msg = None

        # Canny 임계값 파라미터 선언
        self.declare_parameter('threshold1', 100,
            ParameterDescriptor(
                description='Canny threshold1',
                integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]
            )
        )
        self.declare_parameter('threshold2', 200,
            ParameterDescriptor(
                description='Canny threshold2',
                integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]
            )
        )

        # 시뮬레이션 vs real world 토픽 설정
        # 시뮬레이션 OAK-D Lite 카메라 사용 시 아래 주석 해제(브릿지 필요)
        # color_image_topic = '/d435i/color/image_raw'
        # camera_info_topic = '/d435i/color/camera_info'
        # depth_image_topic = '/d435i/depth/image_raw'

        # real world D435i 카메라 사용 시 아래 주석 해제
        color_image_topic = '/camera/camera/color/image_raw'
        camera_info_topic = '/camera/camera/color/camera_info'
        depth_image_topic = '/camera/camera/depth/image_rect_raw'

        self.get_logger().info(f"Subscribing to Color Image: {color_image_topic}")
        self.get_logger().info(f"Subscribing to Camera Info: {camera_info_topic}")
        self.get_logger().info(f"Subscribing to Depth Image (for sync): {depth_image_topic}")

        # QoS 설정 (CameraInfo에 VOLATILE 적용)
        caminfo_qos = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )

        # CameraInfo 구독
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            caminfo_qos
        )

        # Color/Depth 이미지 동기화
        self.color_sub = message_filters.Subscriber(self, Image, color_image_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_image_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.2
        )
        self.ts.registerCallback(self.sync_callback)

        # 퍼블리셔
        self.image_pub = self.create_publisher(Image, '/image_edge', 10)
        self.hole_pub = self.create_publisher(HoleInfo, '/hole_info', 10)

        # 파라미터 변경 콜백
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("Canny Edge Detector node initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        """카메라 내부 파라미터 수신 콜백"""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.last_camera_info_msg = msg
        self.get_logger().info(
            f"Camera info received (HxW: {msg.height}x{msg.width}): "
            f"fx={self.fx:.4f}, fy={self.fy:.4f}, "
            f"cx={self.cx:.4f}, cy={self.cy:.4f}, "
            f"frame_id='{msg.header.frame_id}'"
        )

    def parameter_callback(self, params):
        """파라미터 변경 콜백"""
        return SetParametersResult(successful=True)

    def pixel_to_3d(self, u, v, depth):
        """픽셀(u,v)와 깊이로 카메라 광학 좌표계 3D 포인트 계산"""
        if None in (self.fx, self.fy, self.cx, self.cy):
            self.get_logger().warn(
                "Camera intrinsics not available for 3D projection.",
                throttle_duration_sec=5
            )
            return None
        if self.fx == 0 or self.fy == 0:
            self.get_logger().error(
                "Invalid camera focal length, cannot compute 3D point."
            )
            return None

        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def sync_callback(self, color_msg: Image, depth_msg: Image):
        """동기화된 컬러/깊이 이미지 처리 콜백"""
        # 1) 매 프레임마다 HoleInfo 기본값으로 초기화
        hole_info_msg = HoleInfo()
        hole_info_msg.find_hole = False
        hole_info_msg.passable = False
        hole_info_msg.center = Point(x=0.0, y=0.0, z=0.0)
        hole_info_msg.width = 0.0
        hole_info_msg.height = 0.0

        # 카메라 내부 파라미터 확인
        if None in (self.fx, self.fy, self.cx, self.cy) or self.last_camera_info_msg is None:
            self.get_logger().warn("Waiting for camera intrinsics...", throttle_duration_sec=5)
            self.hole_pub.publish(hole_info_msg)
            return

        # 컬러 이미지 변환
        try:
            cv_color = self.cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            self.hole_pub.publish(hole_info_msg)
            return

        h_img, w_img = cv_color.shape[:2]
        if h_img == 0 or w_img == 0:
            self.get_logger().error("Received empty color image.")
            self.hole_pub.publish(hole_info_msg)
            return

        # 이미지 처리 파이프라인
        gray = cv2.cvtColor(cv_color, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        t1 = self.get_parameter('threshold1').value
        t2 = self.get_parameter('threshold2').value
        edges = cv2.Canny(blurred, t1, t2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result_image = cv_color.copy()
        # 화면 중앙 표시
        cx, cy = w_img // 2, h_img // 2
        cv2.line(result_image, (0, cy), (w_img, cy), (255, 0, 0), 1)
        cv2.line(result_image, (cx, 0), (cx, h_img), (255, 0, 0), 1)
        tol = 23
        cv2.rectangle(
            result_image,
            (cx-tol, cy-tol), (cx+tol, cy+tol),
            (255, 0, 0), 1
        )

        # 2) 컨투어 검사
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
            area = cv2.contourArea(approx)
            if len(approx) == 4 and area > 1000:
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
                # 3D 좌표 계산 (임시 depth=2.0m)

                # 🔴 바운딩 박스 시각화
                cv2.drawContours(result_image, [approx], -1, (0, 255, 0), 2)
                # 🔴 무게 중심 점 표시
                cv2.circle(result_image, (px, py), 5, (0, 0, 255), -1)
                
                depth_est = 2.0
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

        # 3) 결과 이미지 퍼블리시
        try:
            out_img = self.cv_bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
            self.image_pub.publish(out_img)
        except Exception as e:
            self.get_logger().error(f"Publishing edge image error: {e}")

        # 4) HoleInfo 메시지 매 프레임 퍼블리시
        self.hole_pub.publish(hole_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()