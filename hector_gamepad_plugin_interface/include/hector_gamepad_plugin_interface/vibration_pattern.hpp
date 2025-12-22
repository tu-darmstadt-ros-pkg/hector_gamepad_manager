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

struct VibrationPatternDefaults {
  std::vector<double> on_durations_sec;
  std::vector<double> off_durations_sec;
  double intensity{ 1.0 };
  bool cycle{ true };
};

class VibrationPattern
{
public:
  void configure( const rclcpp::Node::SharedPtr &node, const std::string &param_ns,
                  const VibrationPatternDefaults &defaults = VibrationPatternDefaults() )
  {
    node_ = node;
    param_ns_ = param_ns;
    on_durations_sec_ = defaults.on_durations_sec;
    off_durations_sec_ = defaults.off_durations_sec;
    intensity_ = defaults.intensity;
    cycle_ = defaults.cycle;

    declareOrGet( param_ns_ + ".on_durations_sec", on_durations_sec_ );
    declareOrGet( param_ns_ + ".off_durations_sec", off_durations_sec_ );
    declareOrGet( param_ns_ + ".intensity", intensity_ );
    declareOrGet( param_ns_ + ".cycle", cycle_ );

    normalizeDurations();
    updateTotalDuration();
    reset();

    param_callback_ = node_->add_on_set_parameters_callback(
        [this]( const std::vector<rclcpp::Parameter> &params ) {
          return onParameterUpdate( params );
        } );
  }

  void reset()
  {
    if ( !node_ ) {
      return;
    }
    start_time_ = node_->now();
  }

  void setActive( const bool active )
  {
    if ( active && !active_ ) {
      reset();
    }
    active_ = active;
  }

  bool isActive() const { return active_; }

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
  template<typename T>
  void declareOrGet( const std::string &name, T &value )
  {
    if ( !node_->has_parameter( name ) ) {
      node_->declare_parameter<T>( name, value );
    }
    node_->get_parameter( name, value );
  }

  void normalizeDurations()
  {
    for ( auto &value : on_durations_sec_ ) { value = std::max( 0.0, value ); }
    for ( auto &value : off_durations_sec_ ) { value = std::max( 0.0, value ); }
  }

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
