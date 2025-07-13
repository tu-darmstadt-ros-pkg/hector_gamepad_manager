//
// Created by aljoscha-schmidt on 7/13/25.
//

#ifndef VIEWER_PLUGIN_HPP
#define VIEWER_PLUGIN_HPP

#include <hector_rviz_plugins_msgs/srv/move_eye.hpp>
#include <hector_rviz_plugins_msgs/srv/move_eye_and_focus.hpp>
#include <hector_rviz_plugins_msgs/srv/set_view_mode.hpp>
#include <hector_rviz_plugins_msgs/srv/track_frame.hpp>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>

namespace hector_gamepad_manager_plugins
{

class ViewControlPlugin : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  using MoveEye = hector_rviz_plugins_msgs::srv::MoveEye;
  using MoveEyeAndFocus = hector_rviz_plugins_msgs::srv::MoveEyeAndFocus;
  using SetViewMode = hector_rviz_plugins_msgs::srv::SetViewMode;
  using TrackFrame = hector_rviz_plugins_msgs::srv::TrackFrame;
  using Point = geometry_msgs::msg::Point;
  /* Game‑pad plugin API -------------------------------------------------- */
  void initialize( const rclcpp::Node::SharedPtr &node_robot_ns, const rclcpp::Node::SharedPtr &node_operator_ns  ) override;
  std::string getPluginName() override { return "view_control_plugin"; }

  void handleAxis( const std::string &function, double value ) override;
  void handlePress( const std::string &function ) override;
  void handleRelease( const std::string &function ) override;

  /* no special per‑frame “hold” handling; defaults inherited */

  void update() override;
  void activate() override { active_ = true; }
  void deactivate() override;

private:
  /* helper methods ------------------------------------------------------- */
  bool waitForServicesOnce();
  void translateWorld( const Eigen::Vector3d &delta_world );
  void orbit( double d_yaw, double d_pitch );

  /* service clients ------------------------------------------------------ */
  rclcpp::Client<hector_rviz_plugins_msgs::srv::MoveEye>::SharedPtr move_eye_cli_;
  rclcpp::Client<hector_rviz_plugins_msgs::srv::MoveEyeAndFocus>::SharedPtr move_eye_focus_cli_;
  rclcpp::Client<hector_rviz_plugins_msgs::srv::SetViewMode>::SharedPtr mode_cli_;
  rclcpp::Client<hector_rviz_plugins_msgs::srv::TrackFrame>::SharedPtr track_cli_;

  /* runtime state -------------------------------------------------------- */
  bool services_ready_{ false };
  bool tracking_on_{ false };

  Eigen::Vector3d eye_{ 0, 0, 3 }; ///< eye‑focus vector (world)
  Eigen::Vector3d focus{0,0,0};
  geometry_msgs::msg::Point focus_point_{}; ///< world‑frame focus

  /* live axis values ----------------------------------------------------- */
  double orbit_yaw_{ 0.0 };
  double orbit_theta_{ 0.0 };
  double translate_x_{ 0.0 };
  double translate_y_{ 0.0 };
  double zoom_in_{ 0.0 };
  double zoom_out_{ 0.0 };
  int move_z_dir_{ 0 }; ///< +1 up, –1 down, 0 idle

  /* tunable speeds ------------------------------------------------------- */
  double orbit_speed_{ 1.5 };     ///< rad/s
  double zoom_speed_{ 2.0 };      ///< m/s
  double translate_speed_{ 1.0 }; ///< m/s
  double preset_distance_{ 4.0 }; ///< m (distance from base_link in presets)

  /* hacks ------------------------------------------------------- */
  int skip_x_ = 3;
  int counter_ =0;
};
} // namespace hector_gamepad_manager_plugins

#endif // VIEWER_PLUGIN_HPP
