#ifndef HECTOR_GAMEPAD_MANAGER_JOY_SATELLITE_HPP
#define HECTOR_GAMEPAD_MANAGER_JOY_SATELLITE_HPP

#include <chrono>

#include <hector_gamepad_manager_msgs/srv/set_target_robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

namespace hector_gamepad_manager
{
/**
 * @brief Operator-station bridge between the local gamepad and the currently selected robot.
 *
 * Forwards the local joy stream to the selected robot's `joy` topic and bridges that robot's rumble
 * feedback (`<target_robot>/joy_feedback`) back to the local feedback topic. `target_robot` may be
 * an absolute namespace (leading `/`) or one relative to this node's namespace. The selected robot
 * is changed at runtime via the `set_target_robot` service.
 *
 * If the selected robot stops sending feedback while the gamepad is rumbling (e.g. after switching
 * to an idle robot, or the robot dropping out), a watchdog forces the gamepad back to rest after
 * `feedback_timeout_sec`.
 *
 * Thread-safety: all callbacks (joy/feedback subscriptions, watchdog timer, service) share one
 * MutuallyExclusive callback group, so they never run concurrently even under a MultiThreadedExecutor
 * (e.g. when this node is loaded as a component into a shared multi-threaded container). Swapping the
 * target publisher and feedback subscription inside the service callback is therefore race-free.
 */
class JoySatelliteNode : public rclcpp::Node
{
public:
  explicit JoySatelliteNode( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() )
      : rclcpp::Node( "joy_satellite_node", options )
  {
    const std::string input_topic = declare_parameter<std::string>( "input_topic", "joy" );
    const std::string feedback_topic =
        declare_parameter<std::string>( "feedback_topic", "joy/set_feedback" );
    const std::string target_robot = declare_parameter<std::string>( "target_robot", "" );
    double feedback_timeout_sec = declare_parameter<double>( "feedback_timeout_sec", 0.2 );
    if ( feedback_timeout_sec <= 0.0 ) {
      RCLCPP_WARN( get_logger(),
                   "feedback_timeout_sec must be > 0 (got %.3f); falling back to the 0.2 s default",
                   feedback_timeout_sec );
      feedback_timeout_sec = 0.2;
    }

    // All callbacks share one mutually exclusive group so they never run concurrently, even under a
    // MultiThreadedExecutor (e.g. as a component in a shared multi-threaded container). This keeps
    // the publisher/subscription swap in the service callback race-free against the joy/feedback
    // callbacks that read them.
    callback_group_ = create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    // Reliable: this feedback (including the watchdog's stop) is bridged to the local joy node's
    // rumble input. A dropped stop would leave the gamepad rumbling, so don't use best-effort here.
    feedback_publisher_ = create_publisher<sensor_msgs::msg::JoyFeedback>(
        feedback_topic, rclcpp::QoS( 1 ).reliable() );
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
        input_topic, rclcpp::QoS( 1 ),
        [this]( sensor_msgs::msg::Joy::SharedPtr msg ) { forwardJoy( std::move( msg ) ); },
        subscription_options );

    // Rumble watchdog. Re-armed on each non-zero feedback message and fired once if no fresh
    // feedback arrives within the timeout (see bridgeFeedback / onFeedbackTimeout). Starts disarmed.
    const auto feedback_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>( feedback_timeout_sec ) );
    feedback_timeout_timer_ =
        create_wall_timer( feedback_timeout, [this]() { onFeedbackTimeout(); }, callback_group_ );
    feedback_timeout_timer_->cancel();

    // An invalid 'target_robot' makes setTarget's create_publisher throw, aborting construction.
    setTarget( target_robot );

    set_target_service_ = create_service<hector_gamepad_manager_msgs::srv::SetTargetRobot>(
        "set_target_robot",
        [this]( const hector_gamepad_manager_msgs::srv::SetTargetRobot::Request::SharedPtr request,
                hector_gamepad_manager_msgs::srv::SetTargetRobot::Response::SharedPtr response ) {
          handleSetTargetRobot( request, response );
        },
        rclcpp::ServicesQoS(), callback_group_ );
  }

private:
  void forwardJoy( sensor_msgs::msg::Joy::SharedPtr msg )
  {
    last_joy_ = msg;
    if ( joy_publisher_ )
      joy_publisher_->publish( *msg );
  }

  // Point the joy publisher and feedback subscription at a new robot namespace. Publishes one
  // neutral (all-zero) joy message to the previously selected robot so it returns to rest.
  //
  // No observable state is mutated until the new endpoints exist: if robot_namespace does not yield
  // a valid topic name, create_publisher/create_subscription throws an rclcpp name-validation
  // exception BEFORE the old robot is neutralized or the target swapped, leaving the current target
  // fully untouched. Callers translate that throw into a failed service response (or let it abort
  // construction). This replaces hand-rolled namespace validation.
  void setTarget( const std::string &robot_namespace )
  {
    if ( robot_namespace == target_robot_ )
      return;

    if ( robot_namespace.empty() ) {
      // Detach: neutralize the old robot, then drop the endpoints.
      if ( joy_publisher_ && last_joy_ )
        joy_publisher_->publish( makeNeutralJoy() );
      target_robot_ = robot_namespace;
      joy_publisher_.reset();
      feedback_subscription_.reset();
      return;
    }

    // Create (and thereby validate) the new endpoints first; an invalid namespace throws here,
    // before the old robot is neutralized or any member is reassigned.
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;
    auto new_publisher =
        create_publisher<sensor_msgs::msg::Joy>( robot_namespace + "/joy", rclcpp::QoS( 1 ) );
    auto new_feedback_subscription = create_subscription<sensor_msgs::msg::JoyFeedback>(
        robot_namespace + "/joy_feedback", rclcpp::QoS( 1 ),
        [this]( sensor_msgs::msg::JoyFeedback::SharedPtr msg ) {
          bridgeFeedback( std::move( msg ) );
        },
        subscription_options );

    // New target is valid: neutralize the old robot, then swap.
    if ( joy_publisher_ && last_joy_ )
      joy_publisher_->publish( makeNeutralJoy() );
    target_robot_ = robot_namespace;
    joy_publisher_ = std::move( new_publisher );
    feedback_subscription_ = std::move( new_feedback_subscription );
  }

  // Bridge the selected robot's feedback to the local gamepad and (re)arm the watchdog so
  // the gamepad is forced back to rest if the feedback stream stops while still active.
  void bridgeFeedback( sensor_msgs::msg::JoyFeedback::SharedPtr msg )
  {
    feedback_publisher_->publish( *msg );
    if ( msg->intensity > 0.0f )
      feedback_timeout_timer_->reset(); // restart the timeout countdown
    else
      feedback_timeout_timer_->cancel(); // already at rest, no watchdog needed
  }

  // Watchdog: no fresh feedback arrived within the timeout while active → stop the gamepad once.
  void onFeedbackTimeout()
  {
    feedback_timeout_timer_->cancel(); // one-shot until the next non-zero feedback re-arms it
    sensor_msgs::msg::JoyFeedback stop;
    stop.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
    stop.id = 0; // all motors
    stop.intensity = 0.0f;
    feedback_publisher_->publish( stop );
  }

  // A zeroed copy of the last forwarded joy message, so its axis/button counts match the gamepad.
  // Only called after at least one joy has been forwarded (the caller guards on last_joy_).
  sensor_msgs::msg::Joy makeNeutralJoy() const
  {
    sensor_msgs::msg::Joy neutral;
    neutral.header.stamp = now();
    neutral.axes.assign( last_joy_->axes.size(), 0.0f );
    neutral.buttons.assign( last_joy_->buttons.size(), 0 );
    return neutral;
  }

  void handleSetTargetRobot(
      const hector_gamepad_manager_msgs::srv::SetTargetRobot::Request::SharedPtr request,
      hector_gamepad_manager_msgs::srv::SetTargetRobot::Response::SharedPtr response )
  {
    try {
      setTarget( request->robot_namespace );
    } catch ( const std::exception &e ) {
      // Invalid namespace → create_publisher/create_subscription threw; report it and keep target.
      response->success = false;
      response->message =
          "'" + request->robot_namespace + "' is not a valid robot namespace: " + e.what();
      RCLCPP_WARN( get_logger(), "Rejected set_target_robot request for '%s': %s",
                   request->robot_namespace.c_str(), e.what() );
      return;
    }
    response->success = true;
    response->message = "Forwarding gamepad input to '" + target_robot_ + "'";
    RCLCPP_INFO( get_logger(), "Now forwarding gamepad input to robot '%s'", target_robot_.c_str() );
  }

  std::string target_robot_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_publisher_;
  rclcpp::TimerBase::SharedPtr feedback_timeout_timer_;
  rclcpp::Service<hector_gamepad_manager_msgs::srv::SetTargetRobot>::SharedPtr set_target_service_;
};
} // namespace hector_gamepad_manager

#endif // HECTOR_GAMEPAD_MANAGER_JOY_SATELLITE_HPP
