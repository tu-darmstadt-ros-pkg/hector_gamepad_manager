import QtQuick

// Whether a key press reaches the robot, and if not, what is in the way.
//
// Two independent things have to be true: the Joy stream has to be running, and the keyboard has
// to belong to this panel. Both can fail quietly - a topic that resolves to no publisher, a click
// that parked the focus in another dock widget. Keeping the decision in one place means the
// banner, the panel frame and the key handler cannot disagree about it.
//
// Free of ROS and of any UI, like VirtualGamepadState and ProfileSwitcher, so the truth table can
// be tested on its own. The caller phrases the messages.
QtObject {
  //! The Enable button is on, so the Joy stream should be running.
  property bool publishing: false

  //! A publisher exists for the configured joy topic. False means Enable is on but nothing goes
  //! out, which has no other visible symptom in the panel.
  property bool topicValid: false

  //! Any element of the plugin holds the keyboard focus.
  property bool panelFocused: false

  //! The window the panel lives in is the active one. While it is not, key releases never arrive,
  //! so anything held would stay held: treat it as a loss of capture rather than as still driving.
  property bool windowActive: true

  //! The focused element is a text field, which needs the keys for itself.
  property bool typing: false

  //! "off", "invalid", "idle", "typing" or "live", in the order the operator has to fix them:
  //! each state is the first thing still standing between a key press and the robot.
  readonly property string state:
      !publishing ? "off"
    : !topicValid ? "invalid"
    : !(panelFocused && windowActive) ? "idle"
    : typing ? "typing"
    : "live"

  //! True exactly when a key press drives the robot.
  readonly property bool capturing: state === "live"
}
