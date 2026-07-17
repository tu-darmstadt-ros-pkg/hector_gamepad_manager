#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

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

// Find a button/axis entry by its index in an unsorted-by-value message array.
template<typename Vec>
const typename Vec::value_type *findByIndex( const Vec &vec, int index )
{
  for ( const auto &entry : vec )
    if ( entry.index == index )
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
    driving.button_mappings[0] = { drive, { "slow", "Drive slowly" }, {}, {}, {} };
    // Per-event button -> press + double-press actions.
    driving.button_mappings[1] = {
        flipper,
        { "individual_front_flipper_control_mode", "Individual front flipper control" },
        { "sync_front_flippers", "Sync front flippers" },
        {},
        {} };
    // Axis with a description, and an axis without one.
    driving.axis_mappings[0] = { drive, "steer", "Steer" };
    driving.axis_mappings[1] = { drive, "drive", "" };
    configs_["driving"] = driving;

    switches_[6] = { "manipulation", "Switch to manipulation mode" };
    switches_[7] = { "driving", "Switch to driving mode" };
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
  const auto *button = findByIndex( msg.configs[0].buttons, 0 );
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
  const auto *button = findByIndex( msg.configs[0].buttons, 1 );
  ASSERT_NE( button, nullptr );
  ASSERT_EQ( button->actions.size(), 2u );
  EXPECT_EQ( button->actions[0].event, GamepadAction::EVENT_PRESS );
  EXPECT_EQ( button->actions[0].function, "individual_front_flipper_control_mode" );
  EXPECT_EQ( button->actions[0].description, "Individual front flipper control" );
  EXPECT_EQ( button->actions[1].event, GamepadAction::EVENT_DOUBLE_PRESS );
  EXPECT_EQ( button->actions[1].function, "sync_front_flippers" );
  EXPECT_EQ( button->actions[1].description, "Sync front flippers" );
}

TEST_F( GamepadMappingBuilderTest, ButtonsSortedByIndex )
{
  auto msg = build();
  const auto &buttons = msg.configs[0].buttons;
  ASSERT_EQ( buttons.size(), 2u );
  EXPECT_EQ( buttons[0].index, 0 );
  EXPECT_EQ( buttons[1].index, 1 );
}

TEST_F( GamepadMappingBuilderTest, AxesCarryDescriptionsAndEmptyStays )
{
  auto msg = build();
  const auto &axes = msg.configs[0].axes;
  ASSERT_EQ( axes.size(), 2u );
  const auto *steer = findByIndex( axes, 0 );
  ASSERT_NE( steer, nullptr );
  EXPECT_EQ( steer->function, "steer" );
  EXPECT_EQ( steer->description, "Steer" );
  const auto *drive = findByIndex( axes, 1 );
  ASSERT_NE( drive, nullptr );
  EXPECT_EQ( drive->description, "" );
}

TEST_F( GamepadMappingBuilderTest, ConfigSwitchesPopulatedAndSorted )
{
  auto msg = build();
  ASSERT_EQ( msg.config_switches.size(), 2u );
  EXPECT_EQ( msg.config_switches[0].index, 6 );
  EXPECT_EQ( msg.config_switches[0].config, "manipulation" );
  EXPECT_EQ( msg.config_switches[0].description, "Switch to manipulation mode" );
  EXPECT_EQ( msg.config_switches[1].index, 7 );
  EXPECT_EQ( msg.config_switches[1].config, "driving" );
}

TEST_F( GamepadMappingBuilderTest, NullPluginYieldsEmptyName )
{
  configs_["driving"].button_mappings[3] = { nullptr, { "fast", "Drive fast" }, {}, {}, {} };
  auto msg = build();
  const auto *button = findByIndex( msg.configs[0].buttons, 3 );
  ASSERT_NE( button, nullptr );
  EXPECT_EQ( button->plugin, "" );
}
