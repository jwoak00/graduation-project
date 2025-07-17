#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cannyedge_test1_interfaces.msg import HoleInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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

        # 시뮬레이션 vs real world 토픽 설정
        color_image_topic = "/d435i/color/image_raw"
        camera_info_topic = "/d435i/depth/camera_info"
        depth_image_topic = "/d435i/depth/image_raw"

        # real world D435i 카메라 사용 시 아래 주석 해제
        # color_image_topic = "/camera/camera/color/image_raw"
        # camera_info_topic = "/camera/camera/depth/camera_info"
        # depth_image_topic = "/camera/camera/depth/image_rect_raw"

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

        qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE
        )
        # 이미지 퍼블리셔
        self.image_pub = self.create_publisher(Image, "/image_edge", qos_profile)

        self.hole_pub = self.create_publisher(HoleInfo, "/hole_info", 10)
        self.get_logger().info("Canny Edge Detector node initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        # 카메라 내부 파라미터 획득
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

    def detect_holes_morphology_connected_components(self, depth_image):
        """
        모폴로지 연산 + 연결 성분 분석을 통한 구멍 검출
        """
        # 1단계: 유효하지 않은 depth 영역 식별
        invalid_mask = (depth_image <= 0) | np.isnan(depth_image)
        invalid_mask = invalid_mask.astype(np.uint8)
        
        # 2단계: 모폴로지 연산으로 노이즈 제거
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        # Closing: 작은 구멍들을 메워서 노이즈 제거
        cleaned_mask = cv2.morphologyEx(invalid_mask, cv2.MORPH_CLOSE, kernel_close)
        
        # Opening: 작은 돌출부나 노이즈 제거
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_OPEN, kernel_open)
        
        # 3단계: 연결 성분 분석
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            cleaned_mask, connectivity=8
        )
        
        # 4단계: 구멍 후보 필터링 및 분석
        valid_holes = []
        
        for i in range(1, num_labels):  # 0은 배경이므로 제외
            # 통계 정보 추출
            area = stats[i, cv2.CC_STAT_AREA]
            left = stats[i, cv2.CC_STAT_LEFT]
            top = stats[i, cv2.CC_STAT_TOP]
            width = stats[i, cv2.CC_STAT_WIDTH]
            height = stats[i, cv2.CC_STAT_HEIGHT]
            center_x, center_y = centroids[i]
            
            # 면적 기반 필터링
            if area < 500 or area > 50000:  # 너무 작거나 큰 영역 제외
                continue
            
            # 종횡비 필터링
            aspect_ratio = width / height if height > 0 else 0
            if aspect_ratio < 0.2 or aspect_ratio > 5.0:  # 극단적인 비율 제외
                continue
            
            # 원형도 계산 (더 정확한 형태 분석)
            component_mask = (labels == i).astype(np.uint8)
            contours, _ = cv2.findContours(
                component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            
            if len(contours) > 0:
                contour = contours[0]
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # 적절한 원형도를 가진 구멍만 선택
                    if 0.3 < circularity < 1.2:
                        hole_info = {
                            'label': i,
                            'area': area,
                            'center': (int(center_x), int(center_y)),
                            'bbox': (left, top, width, height),
                            'circularity': circularity,
                            'contour': contour,
                            'width': width,
                            'height': height
                        }
                        valid_holes.append(hole_info)
        
        return valid_holes, cleaned_mask

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

        # float 타입으로 변환
        depth_cv = depth_cv.astype(np.float32)

        # 1) Temporal smoothing (EMA 필터) - 활성화
        alpha = 0.3
        if self.prev_depth is None:
            self.prev_depth = depth_cv.copy()
        else:
            depth_cv = alpha * depth_cv + (1 - alpha) * self.prev_depth
            self.prev_depth = depth_cv.copy()

        # 2) Invalid depth 값 제거
        depth_cv[depth_cv <= 0] = np.nan
        depth_cv[depth_cv > 5000] = np.nan  # 5m 이상 제거

        # 3) NaN 값 처리를 위한 안전한 필터링
        valid_mask = ~np.isnan(depth_cv)
        if np.any(valid_mask):
            # NaN이 아닌 값만 필터링
            temp_depth = depth_cv.copy()
            temp_depth[~valid_mask] = 0  # NaN을 0으로 임시 변경
            temp_depth = cv2.medianBlur(temp_depth.astype(np.float32), 5)
            temp_depth = cv2.bilateralFilter(temp_depth, d=5, sigmaColor=0.1, sigmaSpace=5)
            temp_depth[~valid_mask] = np.nan  # 다시 NaN으로 복원
            depth_cv = temp_depth

        # 4) 모폴로지 연산 + 연결 성분 분석으로 구멍 검출
        valid_holes, hole_mask = self.detect_holes_morphology_connected_components(depth_cv)

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
        result_image = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        # 화면 중앙 및 가이드라인
        h, w = result_image.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(result_image, (0, cy), (w, cy), (255, 0, 0), 1)
        cv2.line(result_image, (cx, 0), (cx, h), (255, 0, 0), 1)
        tol = 23
        cv2.rectangle(result_image, (cx-tol, cy-tol), (cx+tol, cy+tol), (255, 0, 0), 1)

        # 6) 검출된 구멍 시각화 및 HoleInfo 작성
        for hole in valid_holes:
            # 구멍 윤곽선 그리기
            cv2.drawContours(result_image, [hole['contour']], -1, (0, 255, 0), 2)
            
            # 구멍 중심점 표시
            center = hole['center']
            cv2.circle(result_image, center, 5, (0, 0, 255), -1)
            
            # 구멍 정보 텍스트 표시
            info_text = f"Area: {hole['area']:.0f}, Circ: {hole['circularity']:.2f}"
            cv2.putText(result_image, info_text, 
                       (center[0] - 50, center[1] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # HoleInfo 메시지 작성 (첫 번째 구멍만)
            if not hole_info_msg.find_hole:
                px, py = center
                depth_est = 3.0  # 고정 depth 사용
                coords = self.pixel_to_3d(px, py, depth_est)
                
                if coords:
                    Xc, Yc, Zc = coords
                    real_w = (hole['width'] / self.fx) * depth_est
                    real_h = (hole['height'] / self.fy) * depth_est
                    
                    hole_info_msg.find_hole = True
                    hole_info_msg.passable = bool(real_w >= 0.5 and real_h >= 0.5)
                    hole_info_msg.center = Point(x=Xc, y=Yc, z=Zc)
                    hole_info_msg.width = real_w
                    hole_info_msg.height = real_h

        # 7) 퍼블리시
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
