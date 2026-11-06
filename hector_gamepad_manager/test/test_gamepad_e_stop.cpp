#include <gmock/gmock.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <hector_gamepad_manager/hector_gamepad_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

// Uses real publishers, unlike the rtest based tests, to see what is published during startup.
class GamepadEStopTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_ = std::make_shared<rclcpp::Node>( "gamepad_e_stop_test_client" );
    e_stop_sub_ = client_->create_subscription<std_msgs::msg::Bool>(
        "/athena/gamepad_e_stop", rclcpp::QoS( 10 ).reliable().transient_local(),
        [this]( const std_msgs::msg::Bool &msg ) {
          std::lock_guard<std::mutex> lock( mutex_ );
          received_.push_back( msg.data );
        } );
    joy_pub_ = client_->create_publisher<sensor_msgs::msg::Joy>( "/athena/joy", 10 );
    executor_.add_node( client_ );
    startManager();
  }

  void TearDown() override { stopSpinning(); }

  void stopSpinning()
  {
    executor_.cancel();
    spin_thread_.join();
  }

  void startManager()
  {
    const auto params =
        std::filesystem::path( __FILE__ ).parent_path() / "config" / "athena_plugin_config.yaml";
    rclcpp::NodeOptions options;
    options.arguments( { "--ros-args", "--params-file", params.string() } );
    node_ = std::make_shared<rclcpp::Node>( "hector_gamepad_manager_test", "athena", options );
    manager_ = std::make_shared<hector_gamepad_manager::HectorGamepadManager>( node_ );
    executor_.add_node( node_ );
    spin_thread_ = std::thread( [this] { executor_.spin(); } );
  }

  void respawnManager()
  {
    // A node destroyed while the executor waits on its entities makes the wait fail.
    stopSpinning();
    executor_.remove_node( node_ );
    manager_.reset();
    node_.reset();
    {
      std::lock_guard<std::mutex> lock( mutex_ );
      received_.clear();
    }
    startManager();
  }

  void pressEStop()
  {
    const auto end = std::chrono::steady_clock::now() + 2s;
    while ( joy_pub_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < end )
      std::this_thread::sleep_for( 10ms );
    sendJoy( true );
    sendJoy( false );
  }

  void sendJoy( bool e_stop_button )
  {
    sensor_msgs::msg::Joy joy;
    joy.axes.assign( 6, 0.0f );
    joy.buttons.assign( 21, 0 );
    joy.buttons[8] = e_stop_button ? 1 : 0; // right stick click in the SDL layout
    joy_pub_->publish( joy );
    std::this_thread::sleep_for( 100ms );
  }

  bool waitForLast( bool value )
  {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while ( std::chrono::steady_clock::now() < end ) {
      {
        std::lock_guard<std::mutex> lock( mutex_ );
        if ( !received_.empty() && received_.back() == value )
          return true;
      }
      std::this_thread::sleep_for( 10ms );
    }
    return false;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<hector_gamepad_manager::HectorGamepadManager> manager_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  std::thread spin_thread_;
  std::mutex mutex_;
  std::vector<bool> received_;
};

TEST_F( GamepadEStopTest, StartAndRespawnEngageAndOnePressReleases )
{
  ASSERT_TRUE( waitForLast( true ) ) << "start did not engage the e-stop";
  pressEStop();
  ASSERT_TRUE( waitForLast( false ) );

  respawnManager();
  EXPECT_TRUE( waitForLast( true ) ) << "respawn did not engage the e-stop";
  pressEStop();
  EXPECT_TRUE( waitForLast( false ) );
}
