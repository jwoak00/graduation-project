#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
from cannyedge_test1_interfaces.msg import HoleInfo  # 사용자 정의 메시지
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import open3d as o3d
from typing import List, Tuple
import traceback

class WindowDetectionNode(Node):
    """
    포인트 클라우드 데이터를 이용하여 벽의 창문(구멍)을 검출하고,
    후보의 위치, 크기 및 통과 가능 여부를 HoleInfo와 Marker 메시지로 퍼블리시하는 노드.
    """
    def __init__(self):
        super().__init__('window_detection_node')
        self.get_logger().info("개선된 Window Detection Node 시작 (드론 미션용)")
        
        # 카메라 내부 파라미터 (CameraInfo)
        self.camera_info = None
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 self.camera_info_callback, 10)

        # QoS 프로파일 설정 (센서 데이터용)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE  # Gazebo depth camera는 보통 Volatile
        )

        # 파라미터 선언
        self.declare_parameter('downsample_ratio', 0.1)      # 포인트 클라우드 다운샘플링 비율
        self.declare_parameter('cluster_tolerance', 0.1)       # DBSCAN 클러스터링 허용 오차 (m)
        self.declare_parameter('min_cluster_size', 100)        # 클러스터 최소 포인트 수
        self.declare_parameter('ransac_distance_threshold', 0.01)  # RANSAC 평면 검출 거리 임계값 (m)
        self.declare_parameter('ransac_n', 3)                  # RANSAC에 필요한 최소 포인트 수
        self.declare_parameter('min_window_width', 0.5)        # 통과 가능한 최소 창문 너비 (m)
        self.declare_parameter('min_window_height', 0.5)       # 통과 가능한 최소 창문 높이 (m)
        self.declare_parameter('valid_min_width', 0.2)         # 유효한 창문 최소 너비
        self.declare_parameter('valid_min_height', 0.2)        # 유효한 창문 최소 높이
        self.declare_parameter('valid_max_width', 2.0)         # 유효한 창문 최대 너비
        self.declare_parameter('valid_max_height', 2.0)        # 유효한 창문 최대 높이

        # 구독자: 포인트 클라우드 데이터 수신
        self.subscriber = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',  # 입력 포인트 클라우드 토픽
            self.cloud_callback,
            qos_profile
        )

        # 발행자: 검출된 구멍(창문) 정보 퍼블리시
        self.publisher = self.create_publisher(HoleInfo, '/hole_info', 10)
        # 발행자: 시각화용 Marker 퍼블리시
        self.marker_publisher = self.create_publisher(Marker, '/window_marker', 10)

        self.get_logger().info("Window Detection Node 초기화 완료.")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        self.get_logger().info(f"CameraInfo received: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

    def cloud_callback(self, msg: PointCloud2):
        self.get_logger().debug("포인트 클라우드 메시지 수신")
        try:
            # 구조화된 배열로 시도 (x, y, z 필드 명시)
            cloud_struct_array = pc2.read_points_numpy(msg, field_names=("x", "y", "z"))
        except Exception as e:
            self.get_logger().error(f"read_points_numpy 호출 실패: {e}\n{traceback.format_exc()}")
            return

        # 구조화된 배열 여부 확인
        if not hasattr(cloud_struct_array.dtype, 'names') or cloud_struct_array.dtype.names is None:
            self.get_logger().info("구조화된 배열이 아님 - 일반 ndarray로 처리")
            points = np.asarray(cloud_struct_array, dtype=np.float32)
            if points.ndim == 1:
                if points.size % 3 == 0:
                    points = points.reshape(-1, 3)
                else:
                    self.get_logger().error("포인트 클라우드 배열의 크기가 3의 배수가 아님")
                    return
            elif points.ndim == 2 and points.shape[1] != 3:
                self.get_logger().error("포인트 클라우드 배열의 열 개수가 3이 아님")
                return
        else:
            valid_mask = (~np.isnan(cloud_struct_array["x"]) &
                          ~np.isnan(cloud_struct_array["y"]) &
                          ~np.isnan(cloud_struct_array["z"]))
            cloud_struct_array = cloud_struct_array[valid_mask]
            if cloud_struct_array.size == 0:
                self.get_logger().warn("NaN 제거 후 포인트 클라우드 데이터 없음.")
                return
            points = np.vstack((
                cloud_struct_array["x"],
                cloud_struct_array["y"],
                cloud_struct_array["z"]
            )).T.astype(np.float32)

        if points.shape[0] == 0:
            self.get_logger().warn("빈 포인트 클라우드 배열 생성됨.")
            return

        # Open3D 포인트 클라우드 객체 생성
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        # 1. 다운샘플링
        downsample_ratio = self.get_parameter('downsample_ratio').value
        if not (0 < downsample_ratio <= 1.0):
            self.get_logger().warn(f"잘못된 downsample_ratio ({downsample_ratio}), 0.1로 설정합니다.")
            downsample_ratio = 0.1
        if len(cloud.points) == 0:
            self.get_logger().warn("다운샘플링 전 원본 포인트 클라우드가 비어있음.")
            return
        filtered_cloud = cloud.random_down_sample(sampling_ratio=downsample_ratio)
        self.get_logger().debug(f"다운샘플링 후 포인트 수: {len(filtered_cloud.points)}")
        if len(filtered_cloud.points) == 0:
            self.get_logger().warn("다운샘플링 결과 빈 포인트 클라우드.")
            return

        # 2. 클러스터링 (DBSCAN)
        cluster_tolerance = self.get_parameter('cluster_tolerance').value
        min_cluster_size = self.get_parameter('min_cluster_size').value
        try:
            labels = np.array(filtered_cloud.cluster_dbscan(
                eps=cluster_tolerance,
                min_points=min_cluster_size,
                print_progress=False
            ))
        except Exception as e:
            self.get_logger().error(f"DBSCAN 클러스터링 실패: {e}\n{traceback.format_exc()}")
            return

        if labels.size == 0:
            self.get_logger().info("클러스터링 결과 없음.")
            return

        unique_labels = np.unique(labels)
        cluster_indices = unique_labels[unique_labels >= 0]
        num_clusters = len(cluster_indices)
        self.get_logger().debug(f"검출된 클러스터 수: {num_clusters}")
        if num_clusters == 0:
            self.get_logger().info("유효한 클러스터 없음.")
            return

        detected_windows: List[Tuple[np.ndarray, float, float, int]] = []

        # 3. 각 클러스터에 대해 평면 검출 (RANSAC)
        for cluster_id in cluster_indices:
            cluster_mask = (labels == cluster_id)
            cluster_points = np.asarray(filtered_cloud.points)[cluster_mask]
            ransac_n = self.get_parameter('ransac_n').value
            if cluster_points.shape[0] < ransac_n:
                self.get_logger().debug(f"클러스터 {cluster_id}: RANSAC 포인트 수 부족 ({cluster_points.shape[0]} < {ransac_n}).")
                continue

            cluster_cloud = o3d.geometry.PointCloud()
            cluster_cloud.points = o3d.utility.Vector3dVector(cluster_points)
            distance_threshold = self.get_parameter('ransac_distance_threshold').value
            try:
                plane_model, inlier_indices = cluster_cloud.segment_plane(
                    distance_threshold=distance_threshold,
                    ransac_n=ransac_n,
                    num_iterations=1000
                )
            except RuntimeError as e:
                self.get_logger().warn(f"클러스터 {cluster_id} RANSAC 실패: {e}")
                continue

            num_inliers = len(inlier_indices)
            if num_inliers < (ransac_n * 5):
                self.get_logger().debug(f"클러스터 {cluster_id}: 평면 인라이어 수 부족 ({num_inliers}개).")
                continue

            plane_points = cluster_points[inlier_indices]
            self.get_logger().debug(f"클러스터 {cluster_id}: 평면 검출됨 (인라이어 {num_inliers}개).")

            # 4. Oriented Bounding Box를 사용한 창문 후보의 크기 및 중심 계산
            try:
                obb = o3d.geometry.OrientedBoundingBox.create_from_points(
                    o3d.utility.Vector3dVector(plane_points)
                )
                # OBB의 extents (3축 길이) 중 가장 작은 값은 두께라고 가정
                sorted_extents = np.sort(obb.extent)  # 오름차순 정렬
                thickness, dim1, dim2 = sorted_extents[0], sorted_extents[1], sorted_extents[2]
                # 일반적으로 창문은 가로가 더 길다고 가정하여, width는 큰 값, height는 작은 값으로 설정
                width_candidate = max(dim1, dim2)
                height_candidate = min(dim1, dim2)
                window_center = obb.center
            except Exception as e:
                self.get_logger().error(f"클러스터 {cluster_id} OBB 계산 오류: {e}\n{traceback.format_exc()}")
                continue

            # 5. 유효성 검사: 유효한 창문 크기 범위 확인
            valid_min_w = self.get_parameter('valid_min_width').value
            valid_min_h = self.get_parameter('valid_min_height').value
            valid_max_w = self.get_parameter('valid_max_width').value
            valid_max_h = self.get_parameter('valid_max_height').value

            if not (valid_min_w <= width_candidate <= valid_max_w and valid_min_h <= height_candidate <= valid_max_h):
                self.get_logger().info(
                    f"클러스터 {cluster_id}: 유효하지 않은 창문 크기 (width={width_candidate:.3f}, height={height_candidate:.3f}). 필터링됨."
                )
                continue

            # 통과 가능 여부 결정 (미션 요구 최소 크기와 비교)
            min_passable_width = self.get_parameter('min_window_width').value
            min_passable_height = self.get_parameter('min_window_height').value
            is_passable = (width_candidate >= min_passable_width and height_candidate >= min_passable_height)

            # 로그 출력 (두번째 코드와 유사한 형식)
            self.get_logger().info(
                f"Center: x={window_center[0]:.3f}, y={window_center[1]:.3f}, z={window_center[2]:.3f}"
            )
            self.get_logger().info(
                f"Size: width={width_candidate:.3f}, height={height_candidate:.3f} (Passable: {is_passable})"
            )

            detected_windows.append((window_center, width_candidate, height_candidate, cluster_id))

        # 6. 최종 창문 후보에 대해 HoleInfo 및 Marker 퍼블리시
        if not detected_windows:
            self.get_logger().info("모든 클러스터 처리 후 유효한 창문 후보 없음.")
            return

        for center, width, height, cluster_id in detected_windows:
            hole_info = HoleInfo()
            hole_info.center = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
            hole_info.width = float(width)
            hole_info.height = float(height)
            hole_info.passable = bool(width >= min_passable_width and height >= min_passable_height)
            self.publisher.publish(hole_info)
            self.get_logger().info(
                f"클러스터 {cluster_id}: HoleInfo 퍼블리시됨 - Center: ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}), "
                f"Size: (width={width:.3f}, height={height:.3f}), Passable: {hole_info.passable}"
            )
            self.publish_marker(msg.header.frame_id, center, width, height, cluster_id)

    def publish_marker(self, frame_id: str, center: np.ndarray, width: float, height: float, marker_id: int):
        """
        검출된 창문 후보를 나타내는 CUBE Marker를 퍼블리시.
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_window"
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(width)
        marker.scale.y = float(height)
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        self.marker_publisher.publish(marker)
        self.get_logger().debug(f"Marker 퍼블리시됨 (ID: {marker_id})")

def main(args=None):
    rclpy.init(args=args)
    node = WindowDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 Window detection node 종료됨.")
    except Exception as e:
        node.get_logger().fatal(f"노드 실행 중 예외 발생: {e}\n{traceback.format_exc()}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
