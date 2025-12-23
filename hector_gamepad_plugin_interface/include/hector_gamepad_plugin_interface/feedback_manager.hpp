#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include "vibration_pattern.hpp"

namespace hector_gamepad_plugin_interface
{
class FeedbackManager
{
public:
  using PatternPtr = std::shared_ptr<VibrationPattern>;

  /**
   * @brief Initialize feedback publishing on a periodic timer.
   *
   * @param node ROS node used to create publisher and timer.
   * @param topic Joy feedback topic to publish to.
   * @param publish_rate_hz Publish frequency (min 10 Hz).
   */
  void initialize( const rclcpp::Node::SharedPtr &node,
                   const std::string &topic = "joy/set_feedback", double publish_rate_hz = 10.0 )
  {
    if ( !node ) {
      return;
    }
    node_ = node;
    publish_rate_hz_ = std::max( publish_rate_hz, 10.0 );
    joy_feedback_publisher_ =
        node_->create_publisher<sensor_msgs::msg::JoyFeedback>( topic, rclcpp::QoS( 1 ) );
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>( 1.0 / publish_rate_hz_ ) );
    publish_timer_ = node_->create_wall_timer( period, [this]() { publishFeedback(); } );
  }

  /**
   * @brief Register a vibration pattern by id.
   */
  void registerVibrationPattern( const std::string &id, const PatternPtr &pattern )
  {
    if ( !pattern ) {
      return;
    }
    vibration_patterns_[id] = pattern;
  }

  /**
   * @brief Unregister a vibration pattern by id.
   */
  void unregisterVibrationPattern( const std::string &id ) { vibration_patterns_.erase( id ); }

  /**
   * @brief Enable or disable a registered pattern.
   */
  void setPatternActive( const std::string &id, const bool active )
  {
    auto it = vibration_patterns_.find( id );
    if ( it == vibration_patterns_.end() ) {
      return;
    }
    if ( auto pattern = it->second.lock() ) {
      pattern->setActive( active );
    } else {
      vibration_patterns_.erase( it );
    }
  }

  /**
   * @brief Compute the max intensity across all active patterns.
   *
   * @return Intensity in [0.0, 1.0], 0.0 once when stopping, or -1.0 if fully idle.
   */
  double getVibrationIntensity()
  {
    double intensity = 0.0;
    for ( auto it = vibration_patterns_.begin(); it != vibration_patterns_.end(); ) {
      auto pattern = it->second.lock();
      if ( !pattern ) {
        it = vibration_patterns_.erase( it );
        continue;
      }
      intensity = std::max( intensity, pattern->getIntensityNow() );
      ++it;
    }
    if ( intensity <= 0.0 && last_intensity_ > 0.0 ) {
      last_intensity_ = intensity;
      return 0.0;
    }
    last_intensity_ = intensity;
    if ( intensity <= 0.0 ) {
      return -1.0;
    }
    return intensity;
  }

  /**
   * @brief Check whether a specific pattern is active.
   */
  bool isActive( const std::string &id ) const
  {
    auto it = vibration_patterns_.find( id );
    if ( it == vibration_patterns_.end() ) {
      return false;
    }
    if ( auto pattern = it->second.lock() ) {
      return pattern->isActive();
    }
    return false;
  }

private:
  /**
   * @brief Publish the current feedback value if active or just transitioned to idle.
   */
  void publishFeedback()
  {
    if ( !joy_feedback_publisher_ ) {
      return;
    }
    const double intensity = getVibrationIntensity();
    if ( intensity < 0.0 ) {
      return;
    }
    sensor_msgs::msg::JoyFeedback feedback;
    feedback.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    feedback.id = 0; // all motors
    feedback.intensity = intensity;
    joy_feedback_publisher_->publish( feedback );
  }

  std::unordered_map<std::string, std::weak_ptr<VibrationPattern>> vibration_patterns_;
  double last_intensity_{ 0.0 };
  double publish_rate_hz_{ 10.0 };
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr joy_feedback_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
} // namespace hector_gamepad_plugin_interface
