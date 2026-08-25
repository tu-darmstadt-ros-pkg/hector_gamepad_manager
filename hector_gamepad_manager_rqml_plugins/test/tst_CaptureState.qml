import QtQuick
import QtTest
import "../qml"

// Green means the keys reach the robot, which is only true if this truth table is, so it is
// checked here rather than by looking at the running plugin.
TestCase {
  id: testCase
  name: "CaptureState"

  CaptureState {
    id: capture
  }

  // Everything in order: the only combination that may report driving.
  function init() {
    capture.publishing = true
    capture.topicValid = true
    capture.panelFocused = true
    capture.windowActive = true
    capture.typing = false
  }

  function test_all_conditions_met_is_live() {
    compare(capture.state, "live")
    verify(capture.capturing)
  }

  // Each state names the first thing standing between a key press and the robot, so the more
  // fundamental problem has to win over one further down the chain.
  function test_state_reports_the_first_blocker() {
    var cases = [
      { publishing: false, topicValid: false, panelFocused: false, typing: true, state: "off" },
      { publishing: true, topicValid: false, panelFocused: false, typing: true, state: "invalid" },
      { publishing: true, topicValid: true, panelFocused: false, typing: true, state: "idle" },
      { publishing: true, topicValid: true, panelFocused: true, typing: true, state: "typing" }
    ]
    for (var i = 0; i < cases.length; ++i) {
      capture.publishing = cases[i].publishing
      capture.topicValid = cases[i].topicValid
      capture.panelFocused = cases[i].panelFocused
      capture.typing = cases[i].typing
      compare(capture.state, cases[i].state)
      verify(!capture.capturing)
    }
  }

  // Key releases are only delivered to the active window, so a deflection held across an alt-tab
  // would never be released. Losing the window has to count as losing the keyboard.
  function test_inactive_window_is_not_capturing() {
    capture.windowActive = false
    compare(capture.state, "idle")
    verify(!capture.capturing)
  }

  // The whole point of the focus scope: the panel drives no matter which of its elements holds
  // the keyboard, and stops as soon as none of them does.
  function test_capture_follows_the_panel_focus() {
    capture.panelFocused = false
    verify(!capture.capturing)
    capture.panelFocused = true
    verify(capture.capturing)
  }
}
