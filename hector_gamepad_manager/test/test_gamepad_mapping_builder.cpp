#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "hector_gamepad_manager/gamepad_buttons.hpp"
#include "hector_gamepad_manager/gamepad_mapping_builder.hpp"

using namespace hector_gamepad_manager;
using hector_gamepad_manager_msgs::msg::GamepadAction;

namespace
{
// Minimal concrete plugin whose only job is to carry a plugin id for the builder to read.
class StubPlugin : public GamepadFunctionPlugin
{
public:
  explicit StubPlugin( const std::string &id ) { setPluginId( id ); }
  void update() override { }
  void activate() override { }
  void deactivate() override { }

protected:
  void initialize( const rclcpp::Node::SharedPtr & ) override { }
};

std::shared_ptr<GamepadFunctionPlugin> makePlugin( const std::string &id )
{
  return std::make_shared<StubPlugin>( id );
}

// Find a button/axis entry by its canonical name - the identifier consumers key off.
template<typename Vec>
const typename Vec::value_type *findByName( const Vec &vec, const std::string &name )
{
  for ( const auto &entry : vec )
    if ( entry.name == name )
      return &entry;
  return nullptr;
}
} // namespace

class GamepadMappingBuilderTest : public ::testing::Test
{
protected:
  std::map<std::string, GamepadConfig> configs_;
  std::array<ConfigSwitch, kNumButtons> switches_;
  std::string default_config_ = "driving";

  void SetUp() override
  {
    auto drive = makePlugin( "hector_gamepad_manager_plugins::DrivePlugin" );
    auto flipper = makePlugin( "hector_gamepad_manager_plugins::FlipperPlugin" );

    GamepadConfig driving;
    driving.description = "Drive the robot and control the flippers";
    // Legacy-flat button -> single on_press action.
    driving.button_mappings[buttonId( "a" )] = { drive, { "slow", "Drive slowly" }, {}, {}, {} };
    // Per-event button -> press + double-press actions.
    driving.button_mappings[buttonId( "b" )] = {
        flipper,
        { "individual_front_flipper_control_mode", "Individual front flipper control" },
        { "sync_front_flippers", "Sync front flippers" },
        {},
        {} };
    // A virtual axis button, and a control only some pads report.
    driving.button_mappings[buttonId( "left_trigger" )] = { drive, { "boost", "Boost" }, {}, {}, {} };
    driving.button_mappings[buttonId( "touchpad" )] = { drive, { "extra", "Extra" }, {}, {}, {} };
    // Axis with a description, and an axis without one.
    driving.axis_mappings[axisId( "left_stick_x" )] = { drive, "steer", "Steer" };
    driving.axis_mappings[axisId( "left_stick_y" )] = { drive, "drive", "" };
    configs_["driving"] = driving;

    switches_[buttonId( "back" )] = { "manipulation", "Switch to manipulation mode" };
    switches_[buttonId( "start" )] = { "driving", "Switch to driving mode" };
  }

  hector_gamepad_manager_msgs::msg::GamepadMapping build()
  {
    return buildGamepadMappingMsg( configs_, switches_, default_config_ );
  }
};

TEST_F( GamepadMappingBuilderTest, TopLevelFields )
{
  auto msg = build();
  EXPECT_EQ( msg.default_config, "driving" );
  ASSERT_EQ( msg.configs.size(), 1u );
  EXPECT_EQ( msg.configs[0].name, "driving" );
  EXPECT_EQ( msg.configs[0].description, "Drive the robot and control the flippers" );
}

TEST_F( GamepadMappingBuilderTest, LegacyFlatButtonYieldsSinglePressAction )
{
  auto msg = build();
  const auto *button = findByName( msg.configs[0].buttons, "a" );
  ASSERT_NE( button, nullptr );
  EXPECT_EQ( button->plugin, "hector_gamepad_manager_plugins::DrivePlugin" );
  ASSERT_EQ( button->actions.size(), 1u );
  EXPECT_EQ( button->actions[0].event, GamepadAction::EVENT_PRESS );
  EXPECT_EQ( button->actions[0].function, "slow" );
  EXPECT_EQ( button->actions[0].description, "Drive slowly" );
}

TEST_F( GamepadMappingBuilderTest, PerEventButtonYieldsDistinctActions )
{
  auto msg = build();
  const auto *button = findByName( msg.configs[0].buttons, "b" );
  ASSERT_NE( button, nullptr );
  ASSERT_EQ( button->actions.size(), 2u );
  EXPECT_EQ( button->actions[0].event, GamepadAction::EVENT_PRESS );
  EXPECT_EQ( button->actions[0].function, "individual_front_flipper_control_mode" );
  EXPECT_EQ( button->actions[0].description, "Individual front flipper control" );
  EXPECT_EQ( button->actions[1].event, GamepadAction::EVENT_DOUBLE_PRESS );
  EXPECT_EQ( button->actions[1].function, "sync_front_flippers" );
  EXPECT_EQ( button->actions[1].description, "Sync front flippers" );
}

// Controls come out in the order they sit on the pad, not in whatever order the unordered_map
// happened to store them.
TEST_F( GamepadMappingBuilderTest, ButtonsAreEmittedInControllerOrder )
{
  auto msg = build();
  const auto &buttons = msg.configs[0].buttons;
  ASSERT_EQ( buttons.size(), 4u );
  EXPECT_EQ( buttons[0].name, "a" );
  EXPECT_EQ( buttons[1].name, "b" );
  EXPECT_EQ( buttons[2].name, "touchpad" );
  EXPECT_EQ( buttons[3].name, "left_trigger" ); // virtual buttons sort after the physical ones
}

TEST_F( GamepadMappingBuilderTest, ButtonsCarryCanonicalNames )
{
  auto msg = build();
  const auto &buttons = msg.configs[0].buttons;
  EXPECT_NE( findByName( buttons, "a" ), nullptr );
  EXPECT_NE( findByName( buttons, "b" ), nullptr );
  // A control only some pads report is named like any other.
  EXPECT_NE( findByName( buttons, "touchpad" ), nullptr );
  // Virtual axis buttons are named by their "axis_buttons" config key.
  EXPECT_NE( findByName( buttons, "left_trigger" ), nullptr );
}

TEST_F( GamepadMappingBuilderTest, AxesCarryCanonicalNames )
{
  auto msg = build();
  EXPECT_EQ( findByName( msg.configs[0].axes, "left_stick_x" )->name, "left_stick_x" );
  EXPECT_EQ( findByName( msg.configs[0].axes, "left_stick_y" )->name, "left_stick_y" );
}

TEST_F( GamepadMappingBuilderTest, AxesCarryDescriptionsAndEmptyStays )
{
  auto msg = build();
  const auto &axes = msg.configs[0].axes;
  ASSERT_EQ( axes.size(), 2u );
  const auto *steer = findByName( axes, "left_stick_x" );
  ASSERT_NE( steer, nullptr );
  EXPECT_EQ( steer->function, "steer" );
  EXPECT_EQ( steer->description, "Steer" );
  const auto *drive = findByName( axes, "left_stick_y" );
  ASSERT_NE( drive, nullptr );
  EXPECT_EQ( drive->description, "" );
}

TEST_F( GamepadMappingBuilderTest, ConfigSwitchesPopulatedAndSorted )
{
  auto msg = build();
  ASSERT_EQ( msg.config_switches.size(), 2u );
  EXPECT_EQ( msg.config_switches[0].name, "back" );
  EXPECT_EQ( msg.config_switches[0].config, "manipulation" );
  EXPECT_EQ( msg.config_switches[0].description, "Switch to manipulation mode" );
  EXPECT_EQ( msg.config_switches[1].name, "start" );
  EXPECT_EQ( msg.config_switches[1].config, "driving" );
}

TEST_F( GamepadMappingBuilderTest, NullPluginYieldsEmptyName )
{
  configs_["driving"].button_mappings[buttonId( "y" )] = {
      nullptr, { "fast", "Drive fast" }, {}, {}, {} };
  auto msg = build();
  const auto *button = findByName( msg.configs[0].buttons, "y" );
  ASSERT_NE( button, nullptr );
  EXPECT_EQ( button->plugin, "" );
}

// "left_trigger" names both an axis and the virtual button derived from it. If the two produced
// the same binding id, one binding's args would overwrite the other's.
TEST( GamepadBindingId, ButtonAndAxisOfTheSameNameDoNotCollide )
{
  EXPECT_NE( buttonBindingId( "driving", "left_trigger" ),
             axisBindingId( "driving", "left_trigger" ) );
}

TEST( GamepadBindingId, DistinctPerConfigAndPerControl )
{
  EXPECT_NE( buttonBindingId( "driving", "a" ), buttonBindingId( "manipulation", "a" ) );
  EXPECT_NE( buttonBindingId( "driving", "a" ), buttonBindingId( "driving", "b" ) );
  EXPECT_EQ( buttonBindingId( "driving", "a" ), buttonBindingId( "driving", "a" ) );
}
