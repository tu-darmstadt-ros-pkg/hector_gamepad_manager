#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_orchestrator/controller_orchestrator.hpp>
#include <hector_gamepad_manager_plugins/flipper_plugin.hpp>
#include <hector_gamepad_plugin_interface/blackboard.hpp>
#include <hector_gamepad_plugin_interface/feedback_manager.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using controller_manager_msgs::srv::SwitchController;
using namespace std::chrono_literals;

// A flipper command sends one FORCE_AUTO switch for its teleop controllers, unless
// switch_controllers is off.
class FlipperControllerSwitchTest : public ::testing::TestWithParam<bool>
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides( { { "flipper_plugin.switch_controllers", GetParam() },
                                   { "flipper_plugin.speed", 1.0 },
                                   { "flipper_plugin.flipper_front_factor", 1.0 } } );
    node_ = std::make_shared<rclcpp::Node>( "flipper_controller_switch_test", options );
    switch_service_ = node_->create_service<SwitchController>(
        "controller_manager/switch_controller",
        [this]( const std::shared_ptr<SwitchController::Request> request,
                const std::shared_ptr<SwitchController::Response> response ) {
          const std::lock_guard<std::mutex> lock( mutex_ );
          requests_.push_back( *request );
          response->ok = true;
        } );
    executor_.add_node( node_ );
    spinner_ = std::thread( [this]() { executor_.spin(); } );

    plugin_ = std::make_shared<hector_gamepad_manager_plugins::FlipperPlugin>();
    plugin_->initializePlugin(
        node_, "hector_gamepad_manager_plugins::FlipperPlugin",
        std::make_shared<hector_gamepad_plugin_interface::Blackboard>(),
        std::make_shared<hector_gamepad_plugin_interface::FeedbackManager>(),
        std::make_shared<controller_orchestrator::ControllerOrchestrator>( node_ ) );
    plugin_->activate();
  }

  void TearDown() override
  {
    executor_.cancel();
    spinner_.join();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  std::shared_ptr<hector_gamepad_manager_plugins::FlipperPlugin> plugin_;
  std::mutex mutex_;
  std::vector<SwitchController::Request> requests_;
};

TEST_P( FlipperControllerSwitchTest, FlipperCommandSwitchesOnlyWhenAllowed )
{
  // The orchestrator sends a switch request once its client has discovered the service.
  const auto requested = [this] {
    const std::lock_guard<std::mutex> lock( mutex_ );
    return !requests_.empty();
  };
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while ( !requested() && std::chrono::steady_clock::now() < deadline ) {
    plugin_->handleAxis( "flipper_front_down", "test_id", 1.0 );
    plugin_->update();
    plugin_->handleAxis( "flipper_front_down", "test_id", 0.0 );
    plugin_->update();
    std::this_thread::sleep_for( 50ms );
  }
  ASSERT_EQ( requested(), GetParam() );
  if ( !GetParam() )
    return;
  const std::lock_guard<std::mutex> lock( mutex_ );
  const auto &request = requests_.front();
  EXPECT_EQ( request.strictness, SwitchController::Request::FORCE_AUTO );
  EXPECT_EQ( request.activate_controllers,
             ( std::vector<std::string>{ "self_collision_avoidance_controller",
                                         "flipper_velocity_controller" } ) );
  EXPECT_TRUE( request.deactivate_controllers.empty() );
}

INSTANTIATE_TEST_SUITE_P( SwitchControllers, FlipperControllerSwitchTest,
                          ::testing::Values( true, false ) );

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
