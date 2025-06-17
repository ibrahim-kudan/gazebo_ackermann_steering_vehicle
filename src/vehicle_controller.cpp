#include "vehicle_controller.hpp"

VehicleController::VehicleController(const double timer_period, const double timeout_duration) : Node{"vehicle_controller"},
                                                                                                 timeout_duration_{timeout_duration},
                                                                                                 last_velocity_time_{get_clock()->now()},
                                                                                                 last_steering_time_{get_clock()->now()},
                                                                                                 body_width_{0.0},
                                                                                                 body_length_{0.0},
                                                                                                 wheel_radius_{0.0},
                                                                                                 wheel_width_{0.0},
                                                                                                 max_steering_angle_{0.0},
                                                                                                 max_velocity_{0.0},
                                                                                                 wheel_base_{0.0},
                                                                                                 track_width_{0.0},
                                                                                                 steering_angle_{0.0},
                                                                                                 velocity_{0.0},
                                                                                                 wheel_angular_velocity_{0.0, 0.0},
                                                                                                 wheel_steering_angle_{0.0, 0.0}
{
  // Declare the used parameters
  declare_parameter<double>("body_width", 0.0);
  declare_parameter<double>("body_length", 0.0);
  declare_parameter<double>("wheel_radius", 0.0);
  declare_parameter<double>("wheel_width", 0.0);
  declare_parameter<double>("max_steering_angle", 0.0);
  declare_parameter<double>("max_velocity", 0.0);
  declare_parameter<std::string>("robot_base_frame", "body_link");
  declare_parameter<std::string>("odom_frame", "odom");

  // Get parameters on startup
  get_parameter("body_width", body_width_);
  get_parameter("body_length", body_length_);
  get_parameter("wheel_radius", wheel_radius_);
  get_parameter("wheel_width", wheel_width_);
  get_parameter("max_steering_angle", max_steering_angle_);
  get_parameter("max_velocity", max_velocity_);
  get_parameter("robot_base_frame", robot_frame_id_);
  get_parameter("odom_frame", odom_frame_id_);

  // Set the track width and wheel base
  track_width_ = body_width_ + (2 * wheel_width_ / 2);
  wheel_base_ = body_length_ - (2 * wheel_radius_);

  // Subscribers
  steering_angle_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "/steering_angle", 10,
      std::bind(&VehicleController::steering_angle_callback, this, std::placeholders::_1));
  velocity_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "/velocity", 10, std::bind(&VehicleController::velocity_callback, this, std::placeholders::_1));
  cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&VehicleController::cmd_vel_callback, this, std::placeholders::_1));
  joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&VehicleController::compute_odom, this, std::placeholders::_1));

  // Publishers
  position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);

  velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10);
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Timer loop
  timer_ = create_wall_timer(std::chrono::duration<double>(timer_period),
                             std::bind(&VehicleController::timer_callback, this));
}

std::pair<double, double> VehicleController::ackermann_steering_angle()
{
  double left_wheel_angle{0.0};
  double right_wheel_angle{0.0};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-3)
  {
    const double sin_angle = sin(abs(steering_angle_));
    const double cos_angle = cos(abs(steering_angle_));

    if (steering_angle_ > 0.0)
    {
      // Left and right wheel angles when steering left
      left_wheel_angle = atan((2 * wheel_base_ * sin_angle) /
                              (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));

      right_wheel_angle = atan((2 * wheel_base_ * sin_angle) /
                               (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));
    }
    else
    {
      // Left and right wheel angles when steering right (mirror left with negative signs)
      left_wheel_angle = -atan((2 * wheel_base_ * sin_angle) /
                               (2 * wheel_base_ * cos_angle + track_width_ * sin_angle));

      right_wheel_angle = -atan((2 * wheel_base_ * sin_angle) /
                                (2 * wheel_base_ * cos_angle - track_width_ * sin_angle));
    }
  }

  return std::make_pair(left_wheel_angle, right_wheel_angle);
}

std::pair<double, double> VehicleController::rear_differential_velocity()
{
  double left_wheel_velocity{velocity_};
  double right_wheel_velocity{velocity_};

  // Steering angle is not zero nor too small
  if (abs(steering_angle_) > 1e-3)
  {
    // Calculate turning radius and angular velocity
    const double turning_radius = wheel_base_ / tan(abs(steering_angle_)); // [m]
    const double vehicle_angular_velocity = velocity_ / turning_radius;    // [rad/s]

    // Compute inner and outer wheel radius
    const double inner_radius = turning_radius - (track_width_ / 2.0);
    const double outer_radius = turning_radius + (track_width_ / 2.0);

    if (steering_angle_ > 0.0)
    {
      // Vehicle turning left
      left_wheel_velocity = vehicle_angular_velocity * inner_radius;
      right_wheel_velocity = vehicle_angular_velocity * outer_radius;
    }
    else
    {
      // Vehicle turning right
      left_wheel_velocity = vehicle_angular_velocity * outer_radius;
      right_wheel_velocity = vehicle_angular_velocity * inner_radius;
    }

    // Determine the maximum wheel velocity
    const double max_wheel_velocity = std::max(abs(left_wheel_velocity),
                                               abs(right_wheel_velocity));

    // Scale both wheel velocities proportionally if the maximum is exceeded
    if (max_wheel_velocity > max_velocity_)
    {
      const double scaling_factor = max_velocity_ / max_wheel_velocity;
      left_wheel_velocity *= scaling_factor;
      right_wheel_velocity *= scaling_factor;
    }
  }

  return std::make_pair(left_wheel_velocity, right_wheel_velocity);
}

void VehicleController::timer_callback()
{
  const auto current_time{get_clock()->now()};
  const auto velocity_elapsed_time{(current_time - last_velocity_time_).nanoseconds()};
  const auto steering_elapsed_time{(current_time - last_steering_time_).nanoseconds()};

  // Reset velocity to zero if timeout
  if (velocity_elapsed_time > timeout_duration_)
  {
    wheel_angular_velocity_ = {0.0, 0.0};
  }

  // Reset steering angle to zero if timeout
  if (steering_elapsed_time > timeout_duration_)
  {
    wheel_steering_angle_ = {0.0, 0.0};
  }

  // Publish steering position
  std_msgs::msg::Float64MultiArray position_msg;
  position_msg.data = wheel_steering_angle_;
  position_publisher_->publish(position_msg);

  // Publish wheels velocity
  std_msgs::msg::Float64MultiArray velocity_msg;
  velocity_msg.data = wheel_angular_velocity_;
  velocity_publisher_->publish(velocity_msg);
}

void VehicleController::steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  update_steering_angle(msg->data);
}

void VehicleController::update_steering_angle(double angle)
{
  last_steering_time_ = get_clock()->now(); // Update timestamp

  if (angle > max_steering_angle_)
  {
    steering_angle_ = max_steering_angle_;
  }
  else if (angle < -max_steering_angle_)
  {
    steering_angle_ = -max_steering_angle_;
  }
  else
  {
    steering_angle_ = angle;
  }

  const auto wheel_angles{ackermann_steering_angle()};

  wheel_steering_angle_ = {wheel_angles.first, wheel_angles.second};
}

void VehicleController::velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  update_velocity(msg->data);
}

void VehicleController::update_velocity(double v)
{
  last_velocity_time_ = get_clock()->now(); // Update timestamp

  if (v > max_velocity_)
  {
    velocity_ = max_velocity_;
  }
  else if (v < -max_velocity_)
  {
    velocity_ = -max_velocity_;
  }
  else
  {
    velocity_ = v;
  }

  const auto wheel_velocity{rear_differential_velocity()};

  // Convert wheel linear velocity to wheel angular velocity
  wheel_angular_velocity_ = {(wheel_velocity.first / wheel_radius_),
                             (wheel_velocity.second / wheel_radius_)};
}

void VehicleController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double v = msg->linear.x;
  const double w = msg->angular.z;
  update_velocity(v);
  if (w == 0)
  {
    update_steering_angle(0);
  }
  else
  {
    const double r = v / w;
    const double steering_angle = atan(wheel_base_ / r);
    RCLCPP_INFO(get_logger(), "New steering anlgle %f, r%f, wheel_base:%f, w:%f", steering_angle, r, wheel_base_, w);
    update_steering_angle(steering_angle);
  }
}

void VehicleController::compute_odom(const sensor_msgs::msg::JointState joints_state)
{
  if (last_joint_state_time_.seconds() == 0)
  {
    last_joint_state_time_ = joints_state.header.stamp;
    return;
  }
  if (joints_state.name.empty())
  {
    return;
  }

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id = robot_frame_id_;
  odom_msg.header.stamp = joints_state.header.stamp;

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = odom_frame_id_;
  tf_msg.child_frame_id = robot_frame_id_;
  tf_msg.header.stamp = joints_state.header.stamp;

  const double dt = (rclcpp::Time(joints_state.header.stamp) - last_joint_state_time_).seconds();
  auto find_joint_index = [&joints_state](const std::string &joint_name)
  {
    auto id = std::find(joints_state.name.begin(), joints_state.name.end(), joint_name) - joints_state.name.begin();
    if (id >= joints_state.name.size())
    {
      throw std::runtime_error("Couldn't find joint " + joint_name);
    }
    return id;
  };

  size_t front_left__wheel_idx, front_right_wheel_idx, front_left_steering_idx, front_right_steering_idx;
  try
  {
    front_left__wheel_idx = find_joint_index("rear_left_wheel_joint");
    front_right_wheel_idx = find_joint_index("rear_right_wheel_joint");
    front_left_steering_idx = find_joint_index("front_left_steering_joint");
    front_right_steering_idx = find_joint_index("front_right_steering_joint");
  }
  catch (const std::runtime_error &error)
  {
    std::string available_joints = "";
    for (const auto &joint : joints_state.name)
    {
      available_joints += joint + ",";
    }
    RCLCPP_ERROR(get_logger(), "Couldn't find the required joints %s. Available joints are %s . no odom will be calculated", error.what(), available_joints.c_str());
    return;
  }

  const double avg_velocity = (joints_state.velocity[front_left__wheel_idx] + joints_state.velocity[front_right_wheel_idx] / 2.0) * wheel_radius_;
  const double avg_steering_angle = joints_state.position[front_left_steering_idx] + joints_state.position[front_right_steering_idx] / 2.0;
  const double angular_velocity = avg_velocity * tan(avg_steering_angle) / wheel_base_;

  yaw_ += angular_velocity * dt;
  x_ += avg_velocity * cos(yaw_) * dt;
  y_ += avg_velocity * sin(yaw_) * dt;

  odom_msg.twist.twist.linear.x = avg_velocity;
  odom_msg.twist.twist.angular.z = angular_velocity;

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom_msg.pose.pose.orientation.w = cos(yaw_ / 2.0);

  RCLCPP_DEBUG(get_logger(), "Avg velocity %f  avg_angle:%f", avg_velocity, avg_steering_angle);
  odom_publisher_->publish(odom_msg);

  tf_msg.transform.translation.x = x_;
  tf_msg.transform.translation.y = y_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf_msg);

  last_joint_state_time_ = joints_state.header.stamp;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleController>());
  rclcpp::shutdown();
  return 0;
}
