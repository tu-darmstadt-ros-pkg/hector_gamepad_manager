// Tests for GamepadFunctionPlugin::ensureControllersActive: level-triggered controller
// reactivation with a retry throttle. Runs against fake in-process controller_manager
// services (no real controller_manager needed).
#include <gtest/gtest.h>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hector_gamepad_plugin_interface/gamepad_plugin_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::SwitchController;
using namespace std::chrono_literals;

namespace
{

// Minimal fake controller manager (all controllers claim the same command interface).
class FakeControllerManager
{
public:
  explicit FakeControllerManager( const rclcpp::Node::SharedPtr &node )
  {
    list_service_ = node->create_service<ListControllers>(
        "controller_manager/list_controllers",
        [this]( const std::shared_ptr<ListControllers::Request>,
                const std::shared_ptr<ListControllers::Response> response ) {
          list_request_count_++;
          std::lock_guard<std::mutex> lock( mutex_ );
          for ( const auto &[name, active] : controller_active_ ) {
            controller_manager_msgs::msg::ControllerState state;
            state.name = name;
            state.state = active ? "active" : "inactive";
            state.required_command_interfaces = { "joint1/position" };
            if ( active )
              state.claimed_interfaces = { "joint1/position" };
            response->controller.push_back( state );
          }
        } );
    switch_service_ = node->create_service<SwitchController>(
        "controller_manager/switch_controller",
        [this]( const std::shared_ptr<SwitchController::Request> request,
                const std::shared_ptr<SwitchController::Response> response ) {
          std::lock_guard<std::mutex> lock( mutex_ );
          for ( const auto &name : request->deactivate_controllers )
            controller_active_[name] = false;
          for ( const auto &name : request->activate_controllers ) {
            controller_active_[name] = true;
            activation_request_count_++;
          }
          response->ok = true;
        } );
  }

  void setController( const std::string &name, const bool active )
  {
    std::lock_guard<std::mutex> lock( mutex_ );
    controller_active_[name] = active;
  }

  int activationRequestCount() const { return activation_request_count_.load(); }
  int listRequestCount() const { return list_request_count_.load(); }

private:
  std::mutex mutex_;
  std::map<std::string, bool> controller_active_;
  std::atomic_int activation_request_count_{ 0 };
  std::atomic_int list_request_count_{ 0 };
  rclcpp::Service<ListControllers>::SharedPtr list_service_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;
};

// Concrete plugin exposing the protected helper under test.
class TestPlugin : public hector_gamepad_plugin_interface::GamepadFunctionPlugin
{
public:
  using GamepadFunctionPlugin::ensureControllersActive;
  void update() override { }
  void activate() override { active_ = true; }
  void deactivate() override { active_ = false; }

protected:
  void initialize( const rclcpp::Node::SharedPtr & ) override { }
};

class EnsureControllersActiveTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if ( !rclcpp::ok() )
      rclcpp::init( 0, nullptr );
    node_ = std::make_shared<rclcpp::Node>( "ensure_controllers_active_test" );
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node( node_ );
    spin_thread_ = std::thread( [this]() { executor_->spin(); } );

    fake_cm_ = std::make_unique<FakeControllerManager>( node_ );
    orchestrator_ = std::make_shared<controller_orchestrator::ControllerOrchestrator>( node_ );
    plugin_ = std::make_shared<TestPlugin>();
    plugin_->initializePlugin(
        node_, "test::TestPlugin", std::make_shared<hector_gamepad_plugin_interface::Blackboard>(),
        std::make_shared<hector_gamepad_plugin_interface::FeedbackManager>(), orchestrator_ );

    const auto probe =
        node_->create_client<ListControllers>( "controller_manager/list_controllers" );
    ASSERT_TRUE( probe->wait_for_service( 5s ) );
  }

  void TearDown() override
  {
    executor_->cancel();
    spin_thread_.join();
  }

  // Wait until the fake received `expected` activation requests (or time out).
  bool waitForActivations( const int expected, const std::chrono::seconds timeout = 5s ) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while ( fake_cm_->activationRequestCount() < expected &&
            std::chrono::steady_clock::now() < deadline ) {
      std::this_thread::sleep_for( 10ms );
    }
    return fake_cm_->activationRequestCount() >= expected;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<FakeControllerManager> fake_cm_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
  std::shared_ptr<TestPlugin> plugin_;
};

} // namespace

TEST_F( EnsureControllersActiveTest, EdgeTriggerActivatesImmediately )
{
  fake_cm_->setController( "ctrl", false );

  plugin_->ensureControllersActive( { "ctrl" }, /*edge_trigger=*/true );

  EXPECT_TRUE( waitForActivations( 1 ) );
  EXPECT_EQ( fake_cm_->activationRequestCount(), 1 );
}

TEST_F( EnsureControllersActiveTest, LevelTriggerRetriesThrottledWhileInactive )
{
  fake_cm_->setController( "ctrl", false );

  // Simulates the plugin update loop running at joystick rate while the stick is deflected:
  // many calls without an edge must collapse into a single activation attempt.
  for ( int i = 0; i < 25; i++ ) {
    plugin_->ensureControllersActive( { "ctrl" }, /*edge_trigger=*/false );
    std::this_thread::sleep_for( 2ms );
  }
  ASSERT_TRUE( waitForActivations( 1 ) );
  std::this_thread::sleep_for( 200ms ); // grace period: no further attempts may sneak in
  EXPECT_EQ( fake_cm_->activationRequestCount(), 1 );

  // The switch already activated the controller in the fake, but the plugin's cached state is
  // only refreshed by the next smart switch (there is no activity topic here). After the retry
  // period, the stale-inactive cache must trigger exactly one more (no-op) switch...
  std::this_thread::sleep_for( 1100ms );
  const int list_requests_before = fake_cm_->listRequestCount();
  plugin_->ensureControllersActive( { "ctrl" }, /*edge_trigger=*/false );
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while ( fake_cm_->listRequestCount() < list_requests_before + 1 &&
          std::chrono::steady_clock::now() < deadline ) {
    std::this_thread::sleep_for( 10ms );
  }
  EXPECT_EQ( fake_cm_->listRequestCount(), list_requests_before + 1 );
  EXPECT_EQ( fake_cm_->activationRequestCount(), 1 ); // analysis found nothing to do

  // ...and once the cache knows the controller is active, further calls do nothing at all.
  std::this_thread::sleep_for( 1100ms );
  const int list_requests_settled = fake_cm_->listRequestCount();
  plugin_->ensureControllersActive( { "ctrl" }, /*edge_trigger=*/false );
  std::this_thread::sleep_for( 500ms );
  EXPECT_EQ( fake_cm_->listRequestCount(), list_requests_settled );
  EXPECT_EQ( fake_cm_->activationRequestCount(), 1 );
}
