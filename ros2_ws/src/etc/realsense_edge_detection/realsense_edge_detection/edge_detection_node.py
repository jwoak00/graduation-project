import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import numpy as np
import cv2
import time

class RealSenseEdgeDetection(Node):
    def __init__(self):
        super().__init__('realsense_edge_detection')
        self.get_logger().info("Starting RealSense Edge Detection Node...")

        # RealSense 카메라 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # 퍼블리셔 생성
        self.publisher = self.create_publisher(Point, 'hole_center', 10)

        # 드론이 통과할 창문 기준 (너비, 높이)
        self.min_width = 100
        self.min_height = 100
        self.max_width = 300
        self.max_height = 300

        # 카메라 매개변수 (Intel RealSense D435 예시)
        self.fx = 277.19  # 초점 거리 (픽셀 단위)

        # FPS 제한 설정
        self.last_frame_time = time.time()
        self.fps_limit = 10  # FPS 제한
        self.previous_rectangle = None  # <- 초기값 추가

    def preprocess_depth(self, depth_image):
        depth_filtered = cv2.medianBlur(depth_image, 5)
        depth_filtered = cv2.GaussianBlur(depth_filtered, (5, 5), 0)
        depth_normalized = cv2.normalize(depth_filtered, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        return cv2.equalizeHist(depth_normalized)

    def calculate_contour_depth(self, contour, depth_image, padding=5):
        x, y, w, h = cv2.boundingRect(contour)
        x1 = max(0, x - padding)
        y1 = max(0, y - padding)
        x2 = min(depth_image.shape[1], x + w + padding)
        y2 = min(depth_image.shape[0], y + h + padding)
        roi = depth_image[y1:y2, x1:x2]
        valid_values = roi[roi > 0]
        if len(valid_values) > 0:
            return np.mean(valid_values)
        else:
            return 0

    def pixels_to_mm(self, pixels, depth):
        """
        픽셀 크기를 mm 단위로 변환.
        """
        return (pixels * depth) / self.fx

    def find_best_rectangle(self, contours, depth_image):
        best_rectangle = None
        best_area = 0
        best_center = None
        best_depth = None
        best_size = (0, 0)  # 너비와 높이를 저장
        best_mm_size = (0.0, 0.0)  # mm 단위 너비와 높이를 저장
        best_area_mm = 0.0  # mm 단위 면적

        for contour in contours:
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.01 * peri, True)

            if len(approx) == 4:  # 사각형 조건
                x, y, w, h = cv2.boundingRect(approx)
                if self.min_width <= w <= self.max_width and self.min_height <= h <= self.max_height:
                    area = w * h
                    center = (x + w // 2, y + h // 2)
                    depth_value = self.calculate_contour_depth(contour, depth_image, padding=5)

                    # mm 단위로 변환
                    width_mm = self.pixels_to_mm(w, depth_value)
                    height_mm = self.pixels_to_mm(h, depth_value)
                    area_mm = width_mm * height_mm

                    if area > best_area:
                        best_area = area
                        best_rectangle = (x, y, w, h)
                        best_center = center
                        best_depth = depth_value
                        best_size = (w, h)
                        best_mm_size = (width_mm, height_mm)
                        best_area_mm = area_mm

        return best_rectangle, best_center, best_depth, best_size, best_mm_size, best_area_mm

    def publish_hole_center(self, center, depth, size, mm_size, area_mm):
        msg = Point()
        msg.x = float(center[0])  # 중심 x 좌표
        msg.y = float(center[1])  # 중심 y 좌표
        msg.z = float(depth)  # Depth 값

        # 콘솔 출력
        self.get_logger().info(
            f"Published Hole Center: x={msg.x}, y={msg.y}, z={msg.z}, "
            f"Width={size[0]}px ({mm_size[0]:.2f}mm), "
            f"Height={size[1]}px ({mm_size[1]:.2f}mm), "
            f"Area={area_mm:.2f}mm²"
        )

        self.publisher.publish(msg)

    def run(self):
        try:
            while rclpy.ok():
                current_time = time.time()
                if current_time - self.last_frame_time < 1 / self.fps_limit:
                    continue
                self.last_frame_time = current_time

                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()

                if not depth_frame:
                    continue

                depth_image = np.asanyarray(depth_frame.get_data())
                preprocessed_depth = self.preprocess_depth(depth_image)

                median_val = np.median(preprocessed_depth)
                lower = int(max(0, 0.5 * median_val))
                upper = int(min(255, 1.5 * median_val))
                edges = cv2.Canny(preprocessed_depth, lower, upper)

                kernel = np.ones((3, 3), np.uint8)
                edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

                contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                best_rectangle, best_center, best_depth, best_size, best_mm_size, best_area_mm = self.find_best_rectangle(contours, depth_image)

                result_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

                if best_rectangle:
                    self.previous_rectangle = best_rectangle
                    x, y, w, h = best_rectangle
                    cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.line(result_image, (x + w // 2, y), (x + w // 2, y + h), (0, 0, 255), 2)
                    self.publish_hole_center(best_center, best_depth, best_size, best_mm_size, best_area_mm)
                elif self.previous_rectangle:
                    x, y, w, h = self.previous_rectangle
                    cv2.rectangle(result_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                cv2.imshow("Filtered Edges with Bounding Box", result_image)

                if cv2.waitKey(1) & 0xFF == 27:
                    break
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    edge_detection_node = RealSenseEdgeDetection()
    edge_detection_node.run()
    edge_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

