#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include <hector_gamepad_manager/hector_gamepad_manager.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Field;
using ::testing::HasSubstr;

constexpr int MAX_BUTTONS = 25;
constexpr int MAX_AXES = 8;

// Verifies that malformed YAML entries (missing function: in legacy format, missing on_press
// in new format) are skipped with a warning instead of throwing or registering broken mappings.
class HectorGamepadManagerMalformedConfigTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<hector_gamepad_manager::HectorGamepadManager> manager_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> sub_joy_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_config_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_press_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_hold_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_probe_release_;

  sensor_msgs::msg::Joy joy_msg_;

  static std::string paramsFilePath()
  {
    auto path = std::filesystem::path( __FILE__ ).parent_path() / "config" /
                "manager_internal_malformed_params.yaml";
    return path.string();
  }

  void SetUp() override
  {
    opts_ = rclcpp::NodeOptions();
    opts_.arguments( { "--ros-args", "--params-file", paramsFilePath() } );

    // Construction must not throw, even though the config has malformed entries.
    ASSERT_NO_THROW( {
      node_ = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_malformed_test", opts_ );
      manager_ = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node_ );
    } );

    sub_joy_ = rtest::findSubscription<sensor_msgs::msg::Joy>( node_, "/ocs/joy" );
    pub_config_ = rtest::findPublisher<std_msgs::msg::String>( node_, "/ocs/joy_teleop_profile" );
    pub_probe_press_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/press" );
    pub_probe_hold_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/hold" );
    pub_probe_release_ =
        rtest::findPublisher<std_msgs::msg::String>( node_, "/athena/test_probe/release" );

    ASSERT_TRUE( sub_joy_ ) << "Joy subscription should be created even when some button mappings "
                               "are malformed.";
    ASSERT_TRUE( pub_config_ );
    ASSERT_TRUE( pub_probe_press_ );

    EXPECT_CALL( *pub_config_, publish( _ ) ).Times( AnyNumber() );

    resetJoy();
  }

  void resetJoy()
  {
    joy_msg_.axes = std::vector<float>( MAX_AXES, 0.0f );
    joy_msg_.buttons = std::vector<int>( MAX_BUTTONS, 0 );
    joy_msg_.axes[2] = 1.0f;
    joy_msg_.axes[5] = 1.0f;
  }

  void setButton( int id, int value ) { joy_msg_.buttons[id] = value; }

  void sendJoy() { sub_joy_->handle_message( joy_msg_ ); }
};

// Test E — Pressing the buttons whose mappings were malformed must not dispatch anything (the
// mappings should have been skipped at load time, not registered with empty function names).
TEST_F( HectorGamepadManagerMalformedConfigTest, MalformedMappingsAreSkipped )
{
  EXPECT_CALL( *pub_probe_press_, publish( _ ) ).Times( 0 );
  EXPECT_CALL( *pub_probe_hold_, publish( _ ) ).Times( 0 );
  EXPECT_CALL( *pub_probe_release_, publish( _ ) ).Times( 0 );

  // Button 0: legacy format with plugin: but no function: — must be skipped.
  setButton( 0, 1 );
  sendJoy();
  setButton( 0, 0 );
  sendJoy();

  // Button 1: new format with on_hold/on_release but no on_press — must be skipped.
  setButton( 1, 1 );
  sendJoy();
  setButton( 1, 0 );
  sendJoy();

  // Button 3: new format with on_double_press but no on_press — must be skipped. Without
  // the strict check, the timeout-flush path would dispatch handlePress("") on this single tap.
  setButton( 3, 1 );
  sendJoy();
  setButton( 3, 0 );
  sendJoy();
}

// Test F — A valid entry coexisting with malformed entries in the same config still works.
TEST_F( HectorGamepadManagerMalformedConfigTest, ValidMappingsStillWorkAlongsideMalformedOnes )
{
  EXPECT_CALL( *pub_probe_press_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "press:probe:" ) ) ) )
      .Times( 1 );
  setButton( 2, 1 );
  sendJoy();
  ::testing::Mock::VerifyAndClearExpectations( pub_probe_press_.get() );

  EXPECT_CALL( *pub_probe_release_,
               publish( Field( &std_msgs::msg::String::data, HasSubstr( "release:probe:" ) ) ) )
      .Times( 1 );
  setButton( 2, 0 );
  sendJoy();
}
