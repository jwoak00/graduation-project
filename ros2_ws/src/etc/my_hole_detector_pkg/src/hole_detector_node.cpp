#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

// Marker
#include <visualization_msgs/msg/marker.hpp>

#include <deque>

struct HoleCandidate
{
  bool detected;
  float cx, cy, cz;
  float width, height, depth;
};

class HoleDetectorNode : public rclcpp::Node
{
public:
  HoleDetectorNode()
  : Node("hole_detector_node")
  {
    // 파라미터 설정 (예시)
    this->declare_parameter<double>("fx", 600.0);
    this->declare_parameter<double>("fy", 600.0);
    this->declare_parameter<double>("cx", 320.0);
    this->declare_parameter<double>("cy", 240.0);

    this->declare_parameter<double>("depth_threshold", 0.2); 
    this->declare_parameter<double>("ransac_distance", 0.01); 
    this->declare_parameter<double>("min_depth", 0.2);
    this->declare_parameter<double>("max_depth", 5.0);

    this->declare_parameter<int>("frame_accumulate", 5); 
    this->declare_parameter<int>("needed_detect_count", 3); 
    this->declare_parameter<int>("min_cluster_size", 50); 
    this->declare_parameter<double>("cluster_tolerance", 0.05);

    // 파라미터 로딩
    fx_ = this->get_parameter("fx").as_double();
    fy_ = this->get_parameter("fy").as_double();
    cx_ = this->get_parameter("cx").as_double();
    cy_ = this->get_parameter("cy").as_double();

    depth_threshold_   = this->get_parameter("depth_threshold").as_double();
    ransac_distance_   = this->get_parameter("ransac_distance").as_double();
    min_depth_         = this->get_parameter("min_depth").as_double();
    max_depth_         = this->get_parameter("max_depth").as_double();

    frame_accumulate_  = this->get_parameter("frame_accumulate").as_int();
    needed_detect_count_=this->get_parameter("needed_detect_count").as_int();
    min_cluster_size_  = this->get_parameter("min_cluster_size").as_int();
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();

    // Depth 구독
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/depth/image_rect_raw",
      10,
      std::bind(&HoleDetectorNode::depthCallback, this, std::placeholders::_1)
    );

    // Marker 퍼블리셔 (RViz 시각화)
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/hole_detector/marker", 10
    );

    RCLCPP_INFO(this->get_logger(), "HoleDetectorNode initialized");
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "depthCallback triggered!");
    // 1) Depth 이미지 (16UC1 -> 32FC1)
    cv::Mat depth_image;
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_ptr->image.convertTo(depth_image, CV_32F, 0.001); 
    }
    else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_image = cv_ptr->image.clone();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Unhandled encoding: %s", msg->encoding.c_str());
      pushFalseCandidate();
      return;
    }

    if(depth_image.empty()){
      pushFalseCandidate();
      return;
    }

    // 2) Depth -> PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(depth_image.rows * depth_image.cols);

    for(int v=0; v<depth_image.rows; v++){
      for(int u=0; u<depth_image.cols; u++){
        float Z = depth_image.at<float>(v,u);
        if(std::isnan(Z) || Z<min_depth_ || Z>max_depth_) 
          continue;
        float X = (u - cx_) * Z / fx_;
        float Y = (v - cy_) * Z / fy_;
        cloud->points.push_back({X,Y,Z});
      }
    }
    cloud->width = cloud->points.size();
    cloud->height=1;

    if(cloud->points.size()<100){
      pushFalseCandidate();
      return;
    }

    // 3) RANSAC 평면 검출
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_distance_);
    seg.setInputCloud(cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size()<50){
      pushFalseCandidate();
      return;
    }

    // line 149에서 'float d' (plane eq aX+bY+cZ + d=0)
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d_plane = coefficients->values[3]; 
    // (원래 d라는 이름이었으나 지금은 중복 피하기 위해 d_plane 등으로 바꿔도 됨)

    // 4) 평면 평균 Z
    double sum_z=0.0;
    for(auto idx: inliers->indices){
      sum_z+= cloud->points[idx].z;
    }
    double wall_depth_avg = sum_z / (double)inliers->indices.size();

    // 5) "벽보다 더 깊은" 점들
    double hole_z = wall_depth_avg + depth_threshold_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_points(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto &pt: cloud->points){
      if(pt.z > hole_z){
        hole_points->points.push_back(pt);
      }
    }
    hole_points->width= hole_points->points.size();
    hole_points->height=1;

    if(hole_points->points.empty()){
      pushFalseCandidate();
      return;
    }

    // 6) 클러스터링
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(hole_points);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(50);
    ec.setSearchMethod(tree);
    ec.setInputCloud(hole_points);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    if(cluster_indices.empty()){
      pushFalseCandidate();
      return;
    }

    // 가장 큰 클러스터
    size_t best_idx=0;
    size_t max_size=0;
    for(size_t i=0; i<cluster_indices.size(); i++){
      if(cluster_indices[i].indices.size()>max_size){
        max_size= cluster_indices[i].indices.size();
        best_idx=i;
      }
    }

    double sumx=0, sumy=0, sumz2=0;
    float minx=9999, miny=9999, minz=9999;
    float maxx=-9999, maxy=-9999, maxz=-9999;

    for(auto idx: cluster_indices[best_idx].indices){
      auto &ppt= hole_points->points[idx];
      sumx+= ppt.x; sumy+= ppt.y; sumz2+= ppt.z;
      if(ppt.x<minx) minx= ppt.x;
      if(ppt.x>maxx) maxx= ppt.x;
      if(ppt.y<miny) miny= ppt.y;
      if(ppt.y>maxy) maxy= ppt.y;
      if(ppt.z<minz) minz= ppt.z;
      if(ppt.z>maxz) maxz= ppt.z;
    }
    double ccount= (double)cluster_indices[best_idx].indices.size();
    float cx= sumx/ccount;
    float cy= sumy/ccount;
    float cz= sumz2/ccount;

    float width_est = std::fabs(maxx-minx);
    float height_est= std::fabs(maxy-miny);
    
    // line 221: 원래 'float d'를 선언했던 부분
    // => 'float depth_est' 로 바꿨고, 사용부분도 동일하게 수정
    float depth_est= std::fabs(maxz-minz);

    // Temporal buffer 업데이트
    HoleCandidate hc;
    hc.detected= true;
    hc.cx= cx; hc.cy= cy; hc.cz= cz;
    hc.width= width_est;
    hc.height= height_est;
    hc.depth= depth_est;

    hole_buffer_.push_back(hc);
    if(hole_buffer_.size()> (size_t)frame_accumulate_){
      hole_buffer_.pop_front();
    }

    // 여러 프레임 중 detected==true가 needed_detect_count_ 이상이면 확정
    int detect_count=0;
    for(auto &h: hole_buffer_){
      if(h.detected) detect_count++;
    }
    if(detect_count>= needed_detect_count_){
      // 구멍 확정: 여러 프레임 평균
      float sx=0, sy=0, sz=0; 
      int valid_detect=0;
      for(auto &h: hole_buffer_){
        if(h.detected){
          sx+= h.cx; 
          sy+= h.cy; 
          sz+= h.cz;
          valid_detect++;
        }
      }
      if(valid_detect>0){
        float mean_cx= sx/(float)valid_detect;
        float mean_cy= sy/(float)valid_detect;
        float mean_cz= sz/(float)valid_detect;

        // Marker 시각화
        publishHoleMarker(mean_cx, mean_cy, mean_cz, width_est, height_est, depth_est, false);
      }
    }
    else{
      // 누적 부족 → 마커 삭제
      publishHoleMarker(0,0,0,0,0,0,true);
    }
  }

  // 구멍 미검출
  void pushFalseCandidate()
  {
    hole_buffer_.push_back({false,0,0,0,0,0,0});
    if(hole_buffer_.size()>(size_t)frame_accumulate_){
      hole_buffer_.pop_front();
    }
  }

  // RViz Marker
  void publishHoleMarker(float cx, float cy, float cz,
                         float w, float h, float dd,
                         bool delete_marker=false)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id= "camera_link";
    marker.header.stamp= this->now();
    marker.ns= "hole_detector";
    marker.id= 0;

    if(delete_marker){
      marker.action= visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(marker);
      return;
    }

    marker.type= visualization_msgs::msg::Marker::CUBE;
    marker.action= visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x= cx;
    marker.pose.position.y= cy;
    marker.pose.position.z= cz;
    marker.pose.orientation.w= 1.0;

    marker.scale.x= (w<0.01f)?0.01f:w;
    marker.scale.y= (h<0.01f)?0.01f:h;
    marker.scale.z= (dd<0.01f)?0.01f:dd;

    marker.color.r= 1.0f; 
    marker.color.g= 0.0f; 
    marker.color.b= 0.0f;
    marker.color.a= 0.5f;

    marker.lifetime= rclcpp::Duration(0,0);
    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // 파라미터
  double fx_, fy_, cx_, cy_;
  double depth_threshold_, ransac_distance_;
  double min_depth_, max_depth_;

  int frame_accumulate_;
  int needed_detect_count_;
  int min_cluster_size_;
  double cluster_tolerance_;

  std::deque<HoleCandidate> hole_buffer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node= std::make_shared<HoleDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
