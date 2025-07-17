#!/usr/bin/env python3
import rclpy, cv2, numpy as np, message_filters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cannyedge_test1_interfaces.msg import HoleInfo
from cv_bridge import CvBridge

class CannyEdgeDetector(Node):
    def __init__(self):
        super().__init__("canny_edge_detector")
        self.get_logger().info("Starting Canny Edge Detector (D435i, ROI=right-¾).")
        self.cv_bridge = CvBridge()

        # --- 카메라 내부 파라미터 -----------------------------
        self.fx = self.fy = self.cx = self.cy = None
        self.prev_depth = None
        # -------------------------------------------------------

        # 🔴시뮬레이션 vs real world 토픽 설정

        # 시뮬레이션 OAK-D Lite 카메라 사용 시 아래 주석 해제(브릿지 필요)
        # color_image_topic = '/d435i/color/image_raw'
        # camera_info_topic = '/d435i/color/camera_info'
        # depth_image_topic = '/d435i/depth/image_raw'

        # topic 설정 (실사 D435i)
        color_topic  = "/camera/camera/color/image_raw"
        depth_topic  = "/camera/camera/depth/image_rect_raw"
        info_topic   = "/camera/camera/depth/camera_info"

        # CameraInfo 구독 (Reliable 그대로)
        self.create_subscription(
            CameraInfo, info_topic, self.camera_info_callback,
            QoSProfile(depth=10,
                    durability=DurabilityPolicy.VOLATILE,
                    reliability=ReliabilityPolicy.RELIABLE))

        # ★ 센서 이미지 QoS: Best Effort로 맞춤
        qos_sensor = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)

        # Color/Depth Subscriber + TimeSync
        color_sub = message_filters.Subscriber(self, Image, color_topic,
                                            qos_profile=qos_sensor)  # ★
        depth_sub = message_filters.Subscriber(self, Image, depth_topic,
                                            qos_profile=qos_sensor)  # ★
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.2)
        self.ts.registerCallback(self.sync_callback)

        self.image_pub = self.create_publisher(Image, "/image_edge", 10)
        self.hole_pub  = self.create_publisher(HoleInfo, "/hole_info", 10)

    # ----------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]

    def pixel_to_3d(self, u, v, depth):
        if None in (self.fx, self.fy, self.cx, self.cy):       return None
        if depth <= 0 or not np.isfinite(depth):               return None
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        return X, Y, depth
    # ----------------------------------------------------------

    def sync_callback(self, color_msg: Image, depth_msg: Image):
        hole_info = HoleInfo(find_hole=False, passable=False,
                            center=Point(), width=0.0, height=0.0)

        if None in (self.fx, self.fy, self.cx, self.cy):
            self.hole_pub.publish(hole_info);  return

        # 이미지 변환
        try:   color = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Color convert: {e}")
            self.hole_pub.publish(hole_info);  return
        try:   depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth convert: {e}")
            self.hole_pub.publish(hole_info);  return

        # ★ ROI: 오른쪽 ¾ 영역만 사용
        x_start = int(color.shape[1] * 0.25)
        color   = color[:, x_start:]
        depth   = depth[:, x_start:]

        # ---------- Pre-filtering ----------
        alpha = 0.2
        if self.prev_depth is None:
            self.prev_depth = depth.copy()
        else:
            depth = alpha * depth + (1-alpha) * self.prev_depth
            self.prev_depth = depth.copy()

        depth = depth.astype(np.float32)
        depth[(depth <= 0) | (depth > 4000)] = np.nan
        depth = cv2.medianBlur(depth, 3)
        depth = cv2.bilateralFilter(depth, 3, 50, 3)

        valid = np.isfinite(depth)
        if np.any(valid):
            d_min, d_max = np.nanmin(depth), np.nanmax(depth)
            norm = ((depth - d_min) / (d_max-d_min) * 255).clip(0,255).astype(np.uint8) \
                if d_max > d_min else np.zeros_like(depth, np.uint8)
        else:
            norm = np.zeros_like(depth, np.uint8)
        vis = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)

        # ---------- 컨투어 검출 ----------
        h, w = vis.shape[:2]
        cx, cy = w//2, h//2
        cv2.line(vis, (0,cy), (w,cy), (255,0,0),1)
        cv2.line(vis, (cx,0), (cx,h), (255,0,0),1)
        cv2.rectangle(vis, (cx-23, cy-23), (cx+23, cy+23), (255,0,0),1)

        hsv  = cv2.cvtColor(vis, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (15,100,100), (35,255,255))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.03*peri, True)
            area = cv2.contourArea(approx)
            x,y,w_box,h_box = cv2.boundingRect(approx)
            ar = w_box/float(h_box) if h_box else 0
            if area>1000 and 0.5<=ar<=2.0 and len(approx)==4:
                M = cv2.moments(approx)
                if M["m00"]==0: continue
                px, py = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                cv2.drawContours(vis, [approx], -1, (0,255,0),2)
                cv2.circle(vis, (px,py),5,(0,0,255),-1)

                depth_est = 1.0  # TODO: margin 이용한 추정값으로 교체
                px_global = px + x_start          # ★ 전체 FOV 좌표
                coords = self.pixel_to_3d(px_global, py, depth_est)
                if coords is None: continue
                Xc,Yc,Zc = coords
                real_w = (w_box/self.fx)*depth_est
                real_h = (h_box/self.fy)*depth_est
                passable = real_w>=0.5 and real_h>=0.5

                hole_info.find_hole = True
                hole_info.passable  = bool(passable)
                hole_info.center    = Point(x=Xc, y=Yc, z=Zc)
                hole_info.width, hole_info.height = real_w, real_h
                break

        # ---------- Publish ----------
        self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(vis, "bgr8"))
        self.hole_pub.publish(hole_info)

def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeDetector()
    try:   rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node();  rclpy.shutdown()

if __name__ == "__main__":
    main()
