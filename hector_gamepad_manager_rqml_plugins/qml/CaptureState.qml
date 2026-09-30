import QtQuick

// Whether key presses reach the robot, and if not, the first thing blocking them.
QtObject {
  //! The Enable button is on.
  property bool publishing: false

  //! A publisher exists for the configured joy topic.
  property bool topicValid: false

  //! Any element of the plugin holds the keyboard focus.
  property bool panelFocused: false

  //! The panel's window is active. Key releases are only delivered to the active window.
  property bool windowActive: true

  //! A text field has the focus.
  property bool typing: false

  //! "off", "invalid", "idle", "typing" or "live", checked in that order.
  readonly property string state:
      !publishing ? "off"
    : !topicValid ? "invalid"
    : !(panelFocused && windowActive) ? "idle"
    : typing ? "typing"
    : "live"

  //! Key presses reach the robot.
  readonly property bool capturing: state === "live"

  //! Joy messages are going out, whatever the keyboard focus. Synthesized presses need only this.
  readonly property bool streaming: publishing && topicValid
}
