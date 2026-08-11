#include "hector_gamepad_manager/gamepad_buttons.hpp"

#include <array>

namespace hector_gamepad_manager
{
namespace
{
// Physical buttons in SDL_GameControllerButton order. Ids past this list are still read off the
// wire but carry no canonical meaning, so they have no name and cannot be bound.
//
// The array index is the wire id, so the ids are spelled out: a miscount would rename a control
// rather than fail to compile.
// clang-format off
constexpr std::array<const char *, 21> kPhysicalButtonNames = {
    /*  0 */ "a",
    /*  1 */ "b",
    /*  2 */ "x",
    /*  3 */ "y",
    /*  4 */ "back",
    /*  5 */ "guide",
    /*  6 */ "start",
    /*  7 */ "left_stick_click",
    /*  8 */ "right_stick_click",
    /*  9 */ "left_bumper",
    /* 10 */ "right_bumper",
    /* 11 */ "dpad_up",
    /* 12 */ "dpad_down",
    /* 13 */ "dpad_left",
    /* 14 */ "dpad_right",
    /* 15 */ "share",    // SDL's MISC1
    /* 16 */ "paddle1",
    /* 17 */ "paddle2",
    /* 18 */ "paddle3",
    /* 19 */ "paddle4",
    /* 20 */ "touchpad",
};

// Axes in SDL_GameControllerAxis order; the index is the wire id, as above.
constexpr std::array<const char *, kNumAxes> kAxisNames = {
    /*  0 */ "left_stick_x",
    /*  1 */ "left_stick_y",
    /*  2 */ "right_stick_x",
    /*  3 */ "right_stick_y",
    /*  4 */ "left_trigger",   // negative on the wire; see isTriggerAxis()
    /*  5 */ "right_trigger",
};
// clang-format on
} // namespace

const std::map<std::string, int> &axisButtonIds()
{
  static const std::map<std::string, int> ids = {
      { "left_stick_left", kVirtualButtonBase + 0 },
      { "left_stick_right", kVirtualButtonBase + 1 },
      { "left_stick_up", kVirtualButtonBase + 2 },
      { "left_stick_down", kVirtualButtonBase + 3 },
      { "left_trigger", kVirtualButtonBase + 4 },
      { "right_stick_left", kVirtualButtonBase + 5 },
      { "right_stick_right", kVirtualButtonBase + 6 },
      { "right_stick_up", kVirtualButtonBase + 7 },
      { "right_stick_down", kVirtualButtonBase + 8 },
      { "right_trigger", kVirtualButtonBase + 9 },
  };
  return ids;
}

std::string buttonName( const int id )
{
  if ( id < 0 || id >= static_cast<int>( kNumButtons ) )
    return "";
  if ( !isAxisButton( id ) ) {
    if ( id < static_cast<int>( kPhysicalButtonNames.size() ) )
      return kPhysicalButtonNames[id];
    return "";
  }
  for ( const auto &[name, virtual_id] : axisButtonIds() )
    if ( virtual_id == id )
      return name;
  return "";
}

std::string axisName( const int id )
{
  if ( id < 0 || id >= static_cast<int>( kNumAxes ) )
    return "";
  return kAxisNames[id];
}

int buttonId( const std::string &name )
{
  for ( std::size_t i = 0; i < kPhysicalButtonNames.size(); ++i )
    if ( name == kPhysicalButtonNames[i] )
      return static_cast<int>( i );
  const auto &ids = axisButtonIds();
  const auto it = ids.find( name );
  return it == ids.end() ? -1 : it->second;
}

int axisId( const std::string &name )
{
  for ( std::size_t i = 0; i < kAxisNames.size(); ++i )
    if ( name == kAxisNames[i] )
      return static_cast<int>( i );
  return -1;
}

bool isTriggerAxis( const int id )
{
  return id == axisId( "left_trigger" ) || id == axisId( "right_trigger" );
}

bool isAxisButton( const int id ) { return id >= static_cast<int>( kVirtualButtonBase ); }

std::string buttonNameList( const bool axis_derived )
{
  std::string names;
  if ( axis_derived ) {
    for ( const auto &[name, id] : axisButtonIds() ) names += name + " ";
  } else {
    for ( const auto *name : kPhysicalButtonNames ) names += std::string( name ) + " ";
  }
  return names;
}

std::string axisNameList()
{
  std::string names;
  for ( const auto *name : kAxisNames ) names += std::string( name ) + " ";
  return names;
}

std::string buttonBindingId( const std::string &config_name, const std::string &button_name )
{
  return config_name + "_button_" + button_name;
}

std::string axisBindingId( const std::string &config_name, const std::string &axis_name )
{
  return config_name + "_axis_" + axis_name;
}
} // namespace hector_gamepad_manager
