#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hector_gamepad_plugin_interface
{

/**
 * @brief Configuration defaults for a vibration pattern.
 */
struct VibrationPatternDefaults {
  std::vector<double> on_durations_sec;
  std::vector<double> off_durations_sec;
  double intensity{ 1.0 };
  bool cycle{ true };
};

/**
 * @brief Time-based rumble pattern with on/off pulses.
 */
class VibrationPattern
{
public:
  /**
   * @brief Configure the pattern parameters and register runtime parameter updates.
   *
   * @param node ROS node used for time and parameters.
   * @param param_ns Parameter namespace for the pattern settings.
   * @param defaults Default values to use when parameters are not set.
   */
  void configure( const rclcpp::Node::SharedPtr &node, const std::string &param_ns,
                  const VibrationPatternDefaults &defaults = VibrationPatternDefaults() )
  {
    node_ = node;
    param_ns_ = param_ns;
    on_durations_sec_ = defaults.on_durations_sec;
    off_durations_sec_ = defaults.off_durations_sec;
    intensity_ = defaults.intensity;
    cycle_ = defaults.cycle;

    on_durations_sec_ = node_->declare_parameter<std::vector<double>>(
        param_ns_ + ".on_durations_sec", on_durations_sec_ );
    off_durations_sec_ = node_->declare_parameter<std::vector<double>>(
        param_ns_ + ".off_durations_sec", off_durations_sec_ );
    intensity_ = node_->declare_parameter<double>( param_ns_ + ".intensity", intensity_ );
    cycle_ = node_->declare_parameter<bool>( param_ns_ + ".cycle", cycle_ );

    normalizeDurations();
    updateTotalDuration();
    reset();

    param_callback_ = node_->add_on_set_parameters_callback(
        [this]( const std::vector<rclcpp::Parameter> &params ) {
          return onParameterUpdate( params );
        } );
  }

  /**
   * @brief Restart the pattern timing reference.
   */
  void reset()
  {
    if ( !node_ ) {
      return;
    }
    start_time_ = node_->now();
  }

  /**
   * @brief Enable or disable the pattern.
   *
   * @param active True to start/continue the pattern, false to stop it.
   */
  void setActive( const bool active )
  {
    if ( active && !active_ ) {
      reset();
    }
    active_ = active;
  }

  /**
   * @brief Check whether the pattern is currently active.
   */
  bool isActive() const { return active_; }

  /**
   * @brief Get the instantaneous intensity based on the current time.
   *
   * @return Intensity in [0.0, 1.0] when active, 0.0 otherwise.
   */
  double getIntensityNow() const
  {
    if ( !active_ || !node_ ) {
      return 0.0;
    }
    if ( total_duration_sec_ <= 0.0 || on_durations_sec_.empty() ) {
      return 0.0;
    }

    double elapsed = ( node_->now() - start_time_ ).seconds();
    if ( elapsed < 0.0 ) {
      elapsed = 0.0;
    }
    if ( cycle_ ) {
      elapsed = std::fmod( elapsed, total_duration_sec_ );
    } else if ( elapsed >= total_duration_sec_ ) {
      return 0.0;
    }

    double cursor = 0.0;
    const size_t pulse_count = on_durations_sec_.size();
    for ( size_t i = 0; i < pulse_count; ++i ) {
      const double on_duration = on_durations_sec_[i];
      const double off_duration = ( i < off_durations_sec_.size() ) ? off_durations_sec_[i] : 0.0;
      if ( elapsed < cursor + on_duration ) {
        return std::clamp( intensity_, 0.0, 1.0 );
      }
      cursor += on_duration;
      if ( elapsed < cursor + off_duration ) {
        return 0.0;
      }
      cursor += off_duration;
    }
    return 0.0;
  }

private:
  /**
   * @brief Clamp on/off durations to non-negative values.
   */
  void normalizeDurations()
  {
    for ( auto &value : on_durations_sec_ ) { value = std::max( 0.0, value ); }
    for ( auto &value : off_durations_sec_ ) { value = std::max( 0.0, value ); }
  }

  /**
   * @brief Recompute the total cycle length from the configured durations.
   */
  void updateTotalDuration()
  {
    total_duration_sec_ = 0.0;
    const size_t pulse_count = on_durations_sec_.size();
    for ( size_t i = 0; i < pulse_count; ++i ) {
      total_duration_sec_ += on_durations_sec_[i];
      if ( i < off_durations_sec_.size() ) {
        total_duration_sec_ += off_durations_sec_[i];
      }
    }
  }

  /**
   * @brief Apply runtime parameter updates for this pattern namespace.
   */
  rcl_interfaces::msg::SetParametersResult
  onParameterUpdate( const std::vector<rclcpp::Parameter> &params )
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool updated = false;
    std::vector<double> next_on_durations = on_durations_sec_;
    std::vector<double> next_off_durations = off_durations_sec_;
    double next_intensity = intensity_;
    bool next_cycle = cycle_;

    for ( const auto &param : params ) {
      const auto &name = param.get_name();
      if ( name == param_ns_ + ".on_durations_sec" ) {
        if ( param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY ) {
          result.successful = false;
          result.reason = "on_durations_sec must be a double array";
          return result;
        }
        next_on_durations = param.as_double_array();
        updated = true;
      } else if ( name == param_ns_ + ".off_durations_sec" ) {
        if ( param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY ) {
          result.successful = false;
          result.reason = "off_durations_sec must be a double array";
          return result;
        }
        next_off_durations = param.as_double_array();
        updated = true;
      } else if ( name == param_ns_ + ".intensity" ) {
        if ( param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ) {
          result.successful = false;
          result.reason = "intensity must be a double";
          return result;
        }
        next_intensity = param.as_double();
        if ( next_intensity < 0.0 || next_intensity > 1.0 ) {
          result.successful = false;
          result.reason = "intensity must be in [0.0, 1.0]";
          return result;
        }
        updated = true;
      } else if ( name == param_ns_ + ".cycle" ) {
        if ( param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL ) {
          result.successful = false;
          result.reason = "cycle must be a bool";
          return result;
        }
        next_cycle = param.as_bool();
        updated = true;
      }
    }

    if ( updated ) {
      on_durations_sec_ = std::move( next_on_durations );
      off_durations_sec_ = std::move( next_off_durations );
      intensity_ = next_intensity;
      cycle_ = next_cycle;
      normalizeDurations();
      updateTotalDuration();
    }

    return result;
  }

  rclcpp::Node::SharedPtr node_;
  std::string param_ns_;
  std::vector<double> on_durations_sec_;
  std::vector<double> off_durations_sec_;
  double intensity_{ 1.0 };
  bool cycle_{ true };
  bool active_{ false };
  double total_duration_sec_{ 0.0 };
  rclcpp::Time start_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

} // namespace hector_gamepad_plugin_interface
