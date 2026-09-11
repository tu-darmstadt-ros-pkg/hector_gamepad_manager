import QtQuick
import QtTest
import "../qml"

TestCase {
  id: testCase
  name: "CaptureState"

  CaptureState {
    id: capture
  }

  // The only combination that reports "live".
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

  // Config switches are synthesized, so they only need the stream.
  function test_streaming_ignores_the_keyboard() {
    capture.panelFocused = false
    capture.typing = true
    verify(capture.streaming, "the stream is up whether or not the panel has the keyboard")
    capture.topicValid = false
    verify(!capture.streaming, "nothing goes out without a publisher")
    capture.topicValid = true
    capture.publishing = false
    verify(!capture.streaming)
  }

  // Key releases are only delivered to the active window.
  function test_inactive_window_is_not_capturing() {
    capture.windowActive = false
    compare(capture.state, "idle")
    verify(!capture.capturing)
  }

  function test_capture_follows_the_panel_focus() {
    capture.panelFocused = false
    verify(!capture.capturing)
    capture.panelFocused = true
    verify(capture.capturing)
  }
}
