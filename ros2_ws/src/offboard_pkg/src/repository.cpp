#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cannyedge_test1_interfaces/msg/hole_info.hpp>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("offboard_control")
  {
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    local_position_subscription_ = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        [this](const VehicleOdometry::SharedPtr msg)
        {
          current_position_[0] = msg->position[0];
          current_position_[1] = msg->position[1];
          current_position_[2] = msg->position[2];

          float dx = current_position_[0] + 3.0f;
          float dz = current_position_[2] + 2.0f;
          float dist = std::sqrt(dx * dx + dz * dz);
          if (dist < 0.2f && !reached_target_position_)
          {
            reached_target_position_ = true;
            align_in_progress_ = true;
            align_completed_ = false;
            fine_movement_in_progress_ = false;
            fixed_y_position_ = current_position_[1]; // Y 고정
            RCLCPP_INFO(this->get_logger(), "Setpoint 위치 도달 및 정렬 시작!");
          }
        });

    hole_info_subscription_ = this->create_subscription<cannyedge_test1_interfaces::msg::HoleInfo>(
        "/hole_info", 10,
        [this](const cannyedge_test1_interfaces::msg::HoleInfo::SharedPtr msg)
        {
          hole_info_msg_ = *msg;
        });

    timer_ = this->create_wall_timer(200ms, std::bind(&OffboardControl::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr local_position_subscription_;
  rclcpp::Subscription<cannyedge_test1_interfaces::msg::HoleInfo>::SharedPtr hole_info_subscription_;

  std::array<float, 3> current_position_{};
  float current_yaw = -1.57f;
  uint64_t offboard_setpoint_counter_ = 0;
  bool reached_target_position_ = false;
  bool align_in_progress_ = false;
  bool align_completed_ = false;
  bool fine_movement_in_progress_ = false;
  int fine_movement_step_count_ = 0;
  const int total_fine_steps_ = 12;
  const float fine_step_distance_ = -0.2f;
  float fixed_y_position_ = 0.0f;
  cannyedge_test1_interfaces::msg::HoleInfo hole_info_msg_;

  void timer_callback()
  {
    if (offboard_setpoint_counter_ == 10)
    {
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
      RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    publish_offboard_control_mode();

    if (!reached_target_position_)
    {
      publish_initial_setpoint();
    }
    else if (align_in_progress_)
    {
      align_and_pass_through();
    }

    if (offboard_setpoint_counter_ < 100)
      offboard_setpoint_counter_++;
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
  {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
  }

  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
  }

  void publish_initial_setpoint()
  {
    TrajectorySetpoint msg{};
    msg.position = {-3.0f, 0.0f, -2.0f};
    msg.yaw = current_yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

  void publish_fixed_trajectory_setpoint(float x, float y, float z, float yaw)
  {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

    class HoleInfo
  {
  public:
    float x;
    float y;
    float z;
  };

  void align_and_pass_through()
  {
    HoleInfo hole_info;
    static HoleInfo old_hole_info;
    old_hole_info.x = hole_info_msg_.center.x;
    old_hole_info.y = hole_info_msg_.center.y;
    old_hole_info.z = hole_info_msg_.center.z;

    if (old_hole_info.x != hole_info_msg_.center.x || old_hole_info.y != hole_info_msg_.center.y || old_hole_info.z != hole_info_msg_.center.z)
    {
      hole_info.x = current_position_[0] + hole_info_msg_.center.x;
      hole_info.y = current_position_[1] + hole_info_msg_.center.y;
      hole_info.z = current_position_[2] + hole_info_msg_.center.z;
    }

    bool is_passable = hole_info_msg_.passable;
    float error_x = hole_info.x - current_position_[0];
    float error_y = hole_info.y - current_position_[1];
    float threshold = 0.01f;
    float step = 0.04f;

    RCLCPP_INFO(this->get_logger(), "Update");
    RCLCPP_INFO(this->get_logger(), "구멍 정보 : %f, %f, %f", hole_info.x, hole_info.y, hole_info.z);
    RCLCPP_INFO(this->get_logger(), "현재 위치 : %f, %f, %f", current_position_[0], current_position_[1], current_position_[2]);
    RCLCPP_INFO(this->get_logger(), "구멍 위치 : %f, %f, %f", hole_info_msg_.center.x, hole_info_msg_.center.y, hole_info_msg_.center.z);
    RCLCPP_INFO(this->get_logger(), "error : %f, %f, %f", hole_info.x - current_position_[0], hole_info.y - current_position_[1], hole_info.z - current_position_[2]);

    if (!align_completed_)
    {
      float target_x = current_position_[0];
      float target_z = current_position_[2];

      // 여기: y축은 절대 안 바꿔! 처음 고정된 값(fixed_y_position_)만 사용
      float target_y = fixed_y_position_;

      if (std::abs(error_x) > threshold)
      {
        target_x += (error_x > 0 ? step : -step);
      }
      if (std::abs(error_y) > threshold)
      {
        target_z += (error_y > 0 ? step : -step);
      }

      publish_fixed_trajectory_setpoint(target_x, target_y, target_z, current_yaw);

      if (std::abs(error_x) <= threshold && std::abs(error_y) <= threshold)
      {
        align_completed_ = true;
        fine_movement_in_progress_ = true;
        fine_movement_step_count_ = 0;
        RCLCPP_INFO(this->get_logger(), "정렬 완료!");
      }
    }
    else if (fine_movement_in_progress_)
    {
      if (fine_movement_step_count_ < total_fine_steps_)
      {
        float next_y = fixed_y_position_ + fine_step_distance_ * (fine_movement_step_count_ + 1);
        publish_fixed_trajectory_setpoint(current_position_[0], next_y, current_position_[2], current_yaw);
        fine_movement_step_count_++;
        RCLCPP_INFO(this->get_logger(), "통과 이동 %d/%d", fine_movement_step_count_, total_fine_steps_);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "모든 이동 완료!");
        align_in_progress_ = false;
        fine_movement_in_progress_ = false;
      }
    }
  }
};

int main(int argc, char *argv[])
{
  std::cout << "Starting offboard control node..." << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}

