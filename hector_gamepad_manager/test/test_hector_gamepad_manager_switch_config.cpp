#include <gmock/gmock.h>
#include <rclcpp/rclcpp.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/service_mock.hpp>
#include <rtest/subscription_mock.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hector_gamepad_manager_msgs/srv/switch_config.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <hector_gamepad_manager/hector_gamepad_manager.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

using hector_gamepad_manager_msgs::srv::SwitchConfig;
using ::testing::AnyNumber;
using ::testing::Field;

constexpr int MAX_BUTTONS = 25;
constexpr int MAX_AXES = 8;

// Exercises the /ocs/switch_config service: retargeting the gamepad to another robot namespace
// caches per-robot state, deactivates the outgoing robot (inert but alive) and re-activates on
// switch-back with its blackboard preserved.
class HectorGamepadManagerSwitchConfigTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions opts_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<hector_gamepad_manager::HectorGamepadManager> manager_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> sub_joy_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_config_;
  std::shared_ptr<rtest::PublisherMock<std_msgs::msg::String>> pub_active_robot_;
  std::shared_ptr<rtest::ServiceMock<SwitchConfig>> switch_service_;
  SwitchConfig::Response last_response_;

  sensor_msgs::msg::Joy joy_msg_;
  std::map<std::string, int> button_map_;
  std::map<std::string, int> axis_map_;

  static std::string paramsFilePath()
  {
    auto path =
        std::filesystem::path( __FILE__ ).parent_path() / "config" / "athena_plugin_config.yaml";
    return path.string();
  }

  void SetUp() override
  {
    opts_ = rclcpp::NodeOptions();
    opts_.arguments( { "--ros-args", "--params-file", paramsFilePath() } );

    node_ = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_switch_test", opts_ );
    manager_ = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node_ );

    sub_joy_ = rtest::findSubscription<sensor_msgs::msg::Joy>( node_, "/ocs/joy" );
    pub_config_ = rtest::findPublisher<std_msgs::msg::String>( node_, "/ocs/joy_teleop_profile" );
    pub_active_robot_ = rtest::findPublisher<std_msgs::msg::String>( node_, "/ocs/active_robot" );
    switch_service_ = rtest::findService<SwitchConfig>( node_, "/ocs/switch_config" );

    ASSERT_TRUE( sub_joy_ );
    ASSERT_TRUE( pub_config_ );
    ASSERT_TRUE( pub_active_robot_ );
    ASSERT_TRUE( switch_service_ );

    // The startup robot must be "athena".
    ASSERT_TRUE( manager_->robotNode() );
    ASSERT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
               "/athena/ocs_gamepad_robot_control" );

    EXPECT_CALL( *pub_config_, publish( ::testing::_ ) ).Times( AnyNumber() );
    EXPECT_CALL( *pub_active_robot_, publish( ::testing::_ ) ).Times( AnyNumber() );

    // The service callback's response is delivered through the mocked send_response; capture it.
    ON_CALL( *switch_service_, send_response( ::testing::_, ::testing::_ ) )
        .WillByDefault( [this]( rmw_request_id_t &, SwitchConfig::Response &resp ) {
          last_response_ = resp;
        } );
    EXPECT_CALL( *switch_service_, send_response( ::testing::_, ::testing::_ ) ).Times( AnyNumber() );

    button_map_ = { { "right_joy", 10 } };
    axis_map_ = { { "left_stick_up_down", 1 } };

    resetJoy();
  }

  void resetJoy()
  {
    joy_msg_.axes = std::vector<float>( MAX_AXES, 0.0f );
    joy_msg_.buttons = std::vector<int>( MAX_BUTTONS, 0 );
    joy_msg_.axes[2] = 1.0f; // LT idle
    joy_msg_.axes[5] = 1.0f; // RT idle
  }

  void setButton( const std::string &button, int value, bool do_reset = false )
  {
    if ( do_reset )
      resetJoy();
    joy_msg_.buttons[button_map_[button]] = value;
  }

  void sendJoy() { sub_joy_->handle_message( joy_msg_ ); }

  // Invoke the switch_config service callback directly (rtest does not spin); returns the response.
  SwitchConfig::Response callSwitch( const std::string &ns, const std::string &cfg,
                                     const std::string &params )
  {
    last_response_ = SwitchConfig::Response{};
    auto request = std::make_shared<SwitchConfig::Request>();
    request->robot_namespace = ns;
    request->config_name = cfg;
    request->plugin_params = params;
    switch_service_->handle_request( std::make_shared<rmw_request_id_t>(), request );
    return last_response_;
  }
};

// Switching to an unknown robot without config_name/plugin_params is rejected.
TEST_F( HectorGamepadManagerSwitchConfigTest, NewRobotRequiresExplicitFields )
{
  auto response = callSwitch( "scout", "", "" );
  EXPECT_FALSE( response.success );
  EXPECT_FALSE( response.message.empty() );
  // Still controlling athena.
  EXPECT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
             "/athena/ocs_gamepad_robot_control" );
}

// Switching to a new robot builds its node, activates it and announces it on /ocs/active_robot.
TEST_F( HectorGamepadManagerSwitchConfigTest, SwitchToNewRobotBuildsActivatesAndAnnounces )
{
  EXPECT_CALL( *pub_active_robot_, publish( Field( &std_msgs::msg::String::data, "scout" ) ) )
      .Times( 1 );

  auto response = callSwitch( "scout", "athena", "athena" );
  EXPECT_TRUE( response.success ) << response.message;

  // The controlled robot is now scout, on its own node with the scout namespace.
  ASSERT_TRUE( manager_->robotNode() );
  EXPECT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
             "/scout/ocs_gamepad_robot_control" );
  auto pub_scout_cmd_vel =
      rtest::findPublisher<geometry_msgs::msg::TwistStamped>( manager_->robotNode(), "/scout/cmd_vel" );
  EXPECT_TRUE( pub_scout_cmd_vel );
}

// Switching away from a robot deactivates it: the drive plugin publishes a final zero command.
TEST_F( HectorGamepadManagerSwitchConfigTest, SwitchAwayDeactivatesOutgoingRobot )
{
  auto pub_athena_cmd_vel =
      rtest::findPublisher<geometry_msgs::msg::TwistStamped>( manager_->robotNode(), "/athena/cmd_vel" );
  ASSERT_TRUE( pub_athena_cmd_vel );

  // On deactivate the drive plugin publishes a zero twist.
  EXPECT_CALL( *pub_athena_cmd_vel, publish( ::testing::_ ) )
      .WillOnce( []( const geometry_msgs::msg::TwistStamped &msg ) {
        EXPECT_EQ( msg.twist.linear.x, 0.0 );
        EXPECT_EQ( msg.twist.angular.z, 0.0 );
      } );

  auto response = callSwitch( "scout", "athena", "athena" );
  EXPECT_TRUE( response.success ) << response.message;
}

// A robot_namespace given with a leading slash ("/scout") is normalized to the bare token so the
// robot node gets a valid single-slash namespace instead of "//scout" (InvalidNamespaceError).
TEST_F( HectorGamepadManagerSwitchConfigTest, LeadingSlashNamespaceNormalized )
{
  EXPECT_CALL( *pub_active_robot_, publish( Field( &std_msgs::msg::String::data, "scout" ) ) )
      .Times( 1 );

  auto response = callSwitch( "/scout", "athena", "athena" );
  EXPECT_TRUE( response.success ) << response.message;

  ASSERT_TRUE( manager_->robotNode() );
  EXPECT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
             "/scout/ocs_gamepad_robot_control" );
}

// An invalid robot_namespace (illegal characters) makes node construction throw; the service must
// report failure instead of letting the exception terminate the manager node.
TEST_F( HectorGamepadManagerSwitchConfigTest, InvalidNamespaceReportsFailureWithoutCrashing )
{
  auto response = callSwitch( "bad ns!", "athena", "athena" );
  EXPECT_FALSE( response.success );
  EXPECT_FALSE( response.message.empty() );
  // Still controlling athena; the manager survived.
  EXPECT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
             "/athena/ocs_gamepad_robot_control" );
}

// An all-empty request keeps the currently controlled robot.
TEST_F( HectorGamepadManagerSwitchConfigTest, EmptyRequestKeepsCurrentRobot )
{
  auto response = callSwitch( "", "", "" );
  EXPECT_TRUE( response.success ) << response.message;
  EXPECT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
             "/athena/ocs_gamepad_robot_control" );
}

// Switching back to a cached robot re-activates it and preserves its blackboard (soft e-stop).
TEST_F( HectorGamepadManagerSwitchConfigTest, SwitchBackPreservesBlackboard )
{
  // Toggle the soft e-stop ON for athena (lazily creates the e-stop publisher and sets true).
  setButton( "right_joy", 1, true );
  sendJoy();
  auto pub_estop =
      rtest::findPublisher<std_msgs::msg::Bool>( manager_->robotNode(), "/athena/gamepad_e_stop" );
  ASSERT_TRUE( pub_estop );
  setButton( "right_joy", 0, true );
  sendJoy();

  // Switch to scout and back to athena.
  ASSERT_TRUE( callSwitch( "scout", "athena", "athena" ).success );
  EXPECT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
             "/scout/ocs_gamepad_robot_control" );
  ASSERT_TRUE( callSwitch( "athena", "", "" ).success );
  EXPECT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
             "/athena/ocs_gamepad_robot_control" );

  // Toggling again must flip from the PRESERVED true -> false. A reset blackboard would flip to
  // true instead, so observing false proves the soft-e-stop value survived the round trip.
  EXPECT_CALL( *pub_estop, publish( Field( &std_msgs::msg::Bool::data, false ) ) ).Times( 1 );
  setButton( "right_joy", 1, true );
  sendJoy();
}

// A rebuild that fails (e.g. a config typo) must be a no-op: the previously working robot stays
// active and controllable instead of being torn down. Regression for discard-before-rebuild.
TEST_F( HectorGamepadManagerSwitchConfigTest, FailedRebuildKeepsWorkingRobot )
{
  auto pub_athena_cmd_vel =
      rtest::findPublisher<geometry_msgs::msg::TwistStamped>( manager_->robotNode(), "/athena/cmd_vel" );
  ASSERT_TRUE( pub_athena_cmd_vel );

  // Request a config change to a non-existent switches file -> rebuild fails.
  auto response = callSwitch( "athena", "does_not_exist", "" );
  EXPECT_FALSE( response.success );
  EXPECT_FALSE( response.message.empty() );

  // Still controlling athena on the same node...
  ASSERT_TRUE( manager_->robotNode() );
  EXPECT_EQ( std::string( manager_->robotNode()->get_fully_qualified_name() ),
             "/athena/ocs_gamepad_robot_control" );

  // ...and it still routes input: driving on the left stick publishes a command (proves not bricked).
  EXPECT_CALL( *pub_athena_cmd_vel, publish( ::testing::_ ) ).Times( ::testing::AtLeast( 1 ) );
  joy_msg_.axes[axis_map_["left_stick_up_down"]] = 1.0f;
  sendJoy();
}

// A plugin-param-only change rebuilds the robot node but must preserve the current within-robot
// config instead of snapping back to the default. Regression for the fresh-RobotControl default.
TEST_F( HectorGamepadManagerSwitchConfigTest, ParamChangePreservesWithinRobotConfig )
{
  // Switch the within-robot config from the default (driving) to manipulation via config-switch
  // button 6 (see test config/athena.yaml).
  joy_msg_.buttons[6] = 1;
  sendJoy();
  joy_msg_.buttons[6] = 0;
  sendJoy();

  // A plugin-param-only change must re-activate manipulation, not fall back to driving.
  EXPECT_CALL( *pub_config_, publish( Field( &std_msgs::msg::String::data, "manipulation" ) ) )
      .Times( ::testing::AtLeast( 1 ) );
  auto response = callSwitch( "athena", "", "athena_alt" );
  EXPECT_TRUE( response.success ) << response.message;
}

// If the startup build fails (e.g. a bad default config), the OCS joy subscription must still be
// created so a later switch_config can recover control. Regression: the subscription used to be
// created only inside the successful-startup branch.
TEST( HectorGamepadManagerStartupFailure, JoySubscriptionSurvivesFailedStartupAndRecovers )
{
  const std::string config_dir =
      ( std::filesystem::path( __FILE__ ).parent_path() / "config" ).string();
  rclcpp::NodeOptions opts;
  opts.parameter_overrides( { { "config_name", "does_not_exist" },
                              { "config_directory", config_dir },
                              { "robot_namespace", "athena" },
                              { "ocs_namespace", "ocs" } } );
  auto node = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_startup_fail_test", opts );
  auto manager = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );

  // Startup build failed: no active robot...
  EXPECT_FALSE( manager->robotNode() );
  // ...but the joy subscription exists so input can be routed once a robot is built.
  ASSERT_TRUE( rtest::findSubscription<sensor_msgs::msg::Joy>( node, "/ocs/joy" ) );

  // Recover via switch_config to a valid robot.
  auto switch_service = rtest::findService<SwitchConfig>( node, "/ocs/switch_config" );
  ASSERT_TRUE( switch_service );
  SwitchConfig::Response resp;
  ON_CALL( *switch_service, send_response( ::testing::_, ::testing::_ ) )
      .WillByDefault( [&resp]( rmw_request_id_t &, SwitchConfig::Response &r ) { resp = r; } );
  EXPECT_CALL( *switch_service, send_response( ::testing::_, ::testing::_ ) ).Times( AnyNumber() );

  auto request = std::make_shared<SwitchConfig::Request>();
  request->robot_namespace = "athena";
  request->config_name = "athena";
  request->plugin_params = "athena";
  switch_service->handle_request( std::make_shared<rmw_request_id_t>(), request );

  EXPECT_TRUE( resp.success ) << resp.message;
  ASSERT_TRUE( manager->robotNode() );
  EXPECT_EQ( std::string( manager->robotNode()->get_fully_qualified_name() ),
             "/athena/ocs_gamepad_robot_control" );
}

// An empty startup config_name means "start idle": no robot is built (no `<dir>/.yaml` read), the
// joy subscription and switch_config service still exist, and a later switch_config loads a robot.
TEST( HectorGamepadManagerStartupFailure, EmptyStartupConfigWaitsForSwitch )
{
  const std::string config_dir =
      ( std::filesystem::path( __FILE__ ).parent_path() / "config" ).string();
  rclcpp::NodeOptions opts;
  opts.parameter_overrides( { { "config_name", "" },
                              { "config_directory", config_dir },
                              { "robot_namespace", "athena" },
                              { "ocs_namespace", "ocs" } } );
  auto node = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_empty_config_test", opts );
  auto manager = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node );

  // No startup config: idle, no robot built...
  EXPECT_FALSE( manager->robotNode() );
  // ...but the joy subscription exists so input can be routed once a robot is built.
  ASSERT_TRUE( rtest::findSubscription<sensor_msgs::msg::Joy>( node, "/ocs/joy" ) );

  // Load a robot via switch_config.
  auto switch_service = rtest::findService<SwitchConfig>( node, "/ocs/switch_config" );
  ASSERT_TRUE( switch_service );
  SwitchConfig::Response resp;
  ON_CALL( *switch_service, send_response( ::testing::_, ::testing::_ ) )
      .WillByDefault( [&resp]( rmw_request_id_t &, SwitchConfig::Response &r ) { resp = r; } );
  EXPECT_CALL( *switch_service, send_response( ::testing::_, ::testing::_ ) ).Times( AnyNumber() );

  auto request = std::make_shared<SwitchConfig::Request>();
  request->robot_namespace = "athena";
  request->config_name = "athena";
  request->plugin_params = "athena";
  switch_service->handle_request( std::make_shared<rmw_request_id_t>(), request );

  EXPECT_TRUE( resp.success ) << resp.message;
  ASSERT_TRUE( manager->robotNode() );
  EXPECT_EQ( std::string( manager->robotNode()->get_fully_qualified_name() ),
             "/athena/ocs_gamepad_robot_control" );
}
