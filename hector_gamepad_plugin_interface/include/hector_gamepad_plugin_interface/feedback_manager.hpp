#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>

#include "vibration_pattern.hpp"

namespace hector_gamepad_plugin_interface
{
class FeedbackManager
{
public:
  using PatternPtr = std::shared_ptr<VibrationPattern>;

  void registerVibrationPattern( const std::string &id, const PatternPtr &pattern )
  {
    if ( !pattern ) {
      return;
    }
    vibration_patterns_[id] = pattern;
  }

  void unregisterVibrationPattern( const std::string &id ) { vibration_patterns_.erase( id ); }

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
  std::unordered_map<std::string, std::weak_ptr<VibrationPattern>> vibration_patterns_;
  double last_intensity_{ 0.0 };
};
} // namespace hector_gamepad_plugin_interface
