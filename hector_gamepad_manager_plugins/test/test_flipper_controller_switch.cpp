#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_orchestrator/controller_orchestrator.hpp>
#include <hector_gamepad_manager_plugins/flipper_plugin.hpp>
#include <hector_gamepad_plugin_interface/blackboard.hpp>
#include <hector_gamepad_plugin_interface/feedback_manager.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

using controller_manager_msgs::srv::ListControllers;
using namespace std::chrono_literals;

// A flipper command activates teleop_controller through the controller orchestrator, whose
// first step is to list the controllers. With switch_controllers false nothing is listed.
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
    list_service_ = node_->create_service<ListControllers>(
        "controller_manager/list_controllers",
        [this]( const std::shared_ptr<ListControllers::Request>,
                const std::shared_ptr<ListControllers::Response> ) { list_requests_++; } );
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
  rclcpp::Service<ListControllers>::SharedPtr list_service_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  std::shared_ptr<hector_gamepad_manager_plugins::FlipperPlugin> plugin_;
  std::atomic<int> list_requests_{ 0 };
};

TEST_P( FlipperControllerSwitchTest, FlipperCommandSwitchesOnlyWhenAllowed )
{
  // Each command after a pause is a switch request, which the orchestrator sends once its
  // client has discovered the service.
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while ( list_requests_ == 0 && std::chrono::steady_clock::now() < deadline ) {
    plugin_->handleAxis( "flipper_front_down", "test_id", 1.0 );
    plugin_->update();
    plugin_->handleAxis( "flipper_front_down", "test_id", 0.0 );
    plugin_->update();
    std::this_thread::sleep_for( 50ms );
  }
  EXPECT_EQ( list_requests_ > 0, GetParam() );
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
