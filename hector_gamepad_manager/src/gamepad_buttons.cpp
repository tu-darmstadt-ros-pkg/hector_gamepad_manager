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

// Axis-derived virtual buttons, indexed by their offset from kVirtualButtonBase. The name a config
// binds, the axis it reads and the deflection that counts as a press sit on one row, so the
// catalog and the Joy adapter that applies it cannot disagree.
constexpr std::array<AxisButton, kNumVirtualButtons> kAxisButtons = { {
    /*  0 */ { "left_stick_left", 0, +1.0f },
    /*  1 */ { "left_stick_right", 0, -1.0f },
    /*  2 */ { "left_stick_up", 1, +1.0f },
    /*  3 */ { "left_stick_down", 1, -1.0f },
    /*  4 */ { "left_trigger", 4, +1.0f },
    /*  5 */ { "right_stick_left", 2, +1.0f },
    /*  6 */ { "right_stick_right", 2, -1.0f },
    /*  7 */ { "right_stick_up", 3, +1.0f },
    /*  8 */ { "right_stick_down", 3, -1.0f },
    /*  9 */ { "right_trigger", 5, +1.0f },
} };
// clang-format on

// Physical ids are the wire ids, so the catalog may only name as many as fit below the range the
// axis-derived buttons start at; past it, isAxisButton() would send a physical button's config
// entry to the wrong section.
static_assert( kPhysicalButtonNames.size() <= kVirtualButtonBase,
               "physical button names would collide with the axis-derived button ids" );

constexpr bool axisButtonsReadKnownAxes()
{
  for ( const auto &axis_button : kAxisButtons )
    if ( axis_button.axis < 0 || axis_button.axis >= static_cast<int>( kNumAxes ) )
      return false;
  return true;
}
static_assert( axisButtonsReadKnownAxes(), "an axis button reads an axis that does not exist" );
} // namespace

const std::array<AxisButton, kNumVirtualButtons> &axisButtons() { return kAxisButtons; }

std::string buttonName( const int id )
{
  if ( id < 0 || id >= static_cast<int>( kNumButtons ) )
    return "";
  if ( isAxisButton( id ) )
    return kAxisButtons[id - kVirtualButtonBase].name;
  if ( id < static_cast<int>( kPhysicalButtonNames.size() ) )
    return kPhysicalButtonNames[id];
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
  for ( std::size_t i = 0; i < kAxisButtons.size(); ++i )
    if ( name == kAxisButtons[i].name )
      return static_cast<int>( kVirtualButtonBase + i );
  return -1;
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
  // Resolved once: this runs for every axis of every Joy message, and axisId() is a name scan.
  static const int left = axisId( "left_trigger" );
  static const int right = axisId( "right_trigger" );
  return id == left || id == right;
}

bool isAxisButton( const int id ) { return id >= static_cast<int>( kVirtualButtonBase ); }

std::string buttonNameList( const bool axis_derived )
{
  std::string names;
  if ( axis_derived ) {
    for ( const auto &axis_button : kAxisButtons ) names += std::string( axis_button.name ) + " ";
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
