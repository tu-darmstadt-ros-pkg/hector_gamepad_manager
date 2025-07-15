//
// Created by aljoscha-schmidt on 7/13/25.
//

#ifndef VIEWER_PLUGIN_HPP
#define VIEWER_PLUGIN_HPP

#include <hector_rviz_plugins_msgs/msg/relative_view_controller_cmd.hpp>
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
  enum PresetDirection {
    FRONT = 0,
    BACK = 1,
    LEFT = 2,
    RIGHT = 3,
    NONE = 4,
  };
  using MoveEye = hector_rviz_plugins_msgs::srv::MoveEye;
  using MoveEyeAndFocus = hector_rviz_plugins_msgs::srv::MoveEyeAndFocus;
  using SetViewMode = hector_rviz_plugins_msgs::srv::SetViewMode;
  using TrackFrame = hector_rviz_plugins_msgs::srv::TrackFrame;
  using Point = geometry_msgs::msg::Point;
  /* Game‑pad plugin API -------------------------------------------------- */
  void initialize( const rclcpp::Node::SharedPtr &node_robot_ns,
                   const rclcpp::Node::SharedPtr &node_operator_ns ) override;
  std::string getPluginName() override { return "view_control_plugin"; }

  void handleAxis( const std::string &function, double value ) override;
  void handlePress( const std::string &function ) override;
  void handleRelease( const std::string &function ) override;
  void handleHold( const std::string &function ) override;

  /* no special per‑frame “hold” handling; defaults inherited */

  void update() override;
  void activate() override { active_ = true; }
  void deactivate() override;
  void reset();

private:
  rclcpp::Publisher<hector_rviz_plugins_msgs::msg::RelativeViewControllerCmd>::SharedPtr view_controller_pub_;
  hector_rviz_plugins_msgs::msg::RelativeViewControllerCmd rviz_cmd_msg;
  rclcpp::Client<hector_rviz_plugins_msgs::srv::MoveEyeAndFocus>::SharedPtr move_eye_and_focus_client_;
  rclcpp::Client<hector_rviz_plugins_msgs::srv::TrackFrame>::SharedPtr track_frame_client_;
  rclcpp::Client<hector_rviz_plugins_msgs::srv::SetViewMode>::SharedPtr set_view_mode_client_;

  /* runtime state -------------------------------------------------------- */
  bool tracking_active_{ false };
  bool switch_to_3d_mode_{ true };
  bool disable_animation_{ true };

  bool activate_tracking_{ false }; // request to activate tracking
  bool activate_2d_mode_{ false };

  PresetDirection preset_direction_{ PresetDirection::FRONT };

  /* tunable speeds ------------------------------------------------------- */
  double orbit_speed_{ 1.5 };     ///< rad/s
  double zoom_speed_{ 2.0 };      ///< m/s
  double translate_speed_{ 1.0 }; ///< m/s
  double preset_distance_{ 2.0 }; ///< m (distance from base_link in presets)
  double preset_eye_height_{ 1.5 };
  double preset_focus_height_{ 0.25 };
  std::string tracked_frame_;
  bool interacted_{ false };
};
} // namespace hector_gamepad_manager_plugins

#endif // VIEWER_PLUGIN_HPP
