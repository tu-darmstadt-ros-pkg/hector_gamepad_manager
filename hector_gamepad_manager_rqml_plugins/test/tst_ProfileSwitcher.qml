import QtQuick
import QtTest
import "../qml"

TestCase {
  id: testCase
  name: "ProfileSwitcher"
  when: windowShown

  property var presses: []
  property var succeeded: []
  property var failures: []

  ProfileSwitcher {
    id: switcher
    // Short enough to let a test actually wait for it.
    timeoutMs: 120
    configSwitches: [{ config: "driving", name: "back" },
                     { config: "manipulation", name: "start" },
                     { config: "inspection", name: "left_trigger" }]
    pressableButtons: ({ "back": 6, "start": 7 })
    activeProfile: "driving"

    onPressRequested: name => testCase.presses.push(name)
    onSucceeded: config => testCase.succeeded.push(config)
    onFailed: (config, reason, detail) =>
      testCase.failures.push({ config: config, reason: reason, detail: detail })
  }

  function init() {
    switcher.abort()
    switcher.activeProfile = "driving"
    presses = []
    succeeded = []
    failures = []
  }

  function test_request_presses_the_reserved_button() {
    verify(switcher.request("manipulation"))
    compare(presses, ["start"])
    compare(switcher.pendingProfile, "manipulation")
    verify(switcher.busy)
  }

  // The whole point of the rework: the robot reporting the requested profile clears the request,
  // so nothing is left pinned to a stale selection.
  function test_the_profile_message_confirms_the_request() {
    switcher.request("manipulation")
    switcher.activeProfile = "manipulation"
    compare(succeeded, ["manipulation"])
    compare(switcher.pendingProfile, "")
    verify(!switcher.busy)
    compare(failures.length, 0)
  }

  // A confirmation must not later be undone by the timeout that was running for it.
  function test_a_confirmed_switch_does_not_also_time_out() {
    switcher.request("manipulation")
    switcher.activeProfile = "manipulation"
    wait(switcher.timeoutMs * 2)
    compare(failures.length, 0, "the timeout fired after the switch was already confirmed")
    compare(succeeded, ["manipulation"])
  }

  function test_no_answer_times_out() {
    switcher.request("manipulation")
    tryCompare(testCase, "failures", [{ config: "manipulation", reason: "timeout",
                                        detail: "driving" }], 1000)
    compare(switcher.pendingProfile, "", "a timed-out request must not stay pending")
    compare(succeeded.length, 0)
  }

  // A profile the virtual gamepad cannot reach has to say so rather than doing nothing, which is
  // what the old code did.
  function test_a_switch_on_a_virtual_axis_button_reports_why() {
    verify(!switcher.request("inspection"))
    compare(failures.length, 1)
    compare(failures[0].reason, "unpressable")
    compare(failures[0].detail, "left_trigger")
    compare(presses.length, 0)
    verify(!switcher.busy)
  }

  function test_an_unbound_profile_reports_why() {
    verify(!switcher.request("nonexistent"))
    compare(failures.length, 1)
    compare(failures[0].reason, "unbound")
    compare(presses.length, 0)
  }

  function test_requesting_the_active_profile_does_nothing() {
    verify(!switcher.request("driving"))
    compare(presses.length, 0)
    compare(failures.length, 0, "already being there is not a failure")
  }

  function test_a_second_request_is_ignored_while_one_is_in_flight() {
    verify(switcher.request("manipulation"))
    verify(!switcher.request("inspection"))
    compare(presses, ["start"])
    compare(switcher.pendingProfile, "manipulation")
  }

  // Someone switching on a real gamepad moves the active profile without us asking; that must not
  // be reported as our success.
  function test_an_unrequested_profile_change_is_not_a_confirmation() {
    switcher.activeProfile = "manipulation"
    compare(succeeded.length, 0)
    compare(switcher.pendingProfile, "")
  }

  // A change to some third profile while we wait leaves the request pending until it times out.
  function test_a_different_profile_does_not_confirm_the_request() {
    switcher.request("manipulation")
    switcher.activeProfile = "inspection"
    compare(succeeded.length, 0)
    compare(switcher.pendingProfile, "manipulation")
  }

  function test_abort_drops_the_request_silently() {
    switcher.request("manipulation")
    switcher.abort()
    compare(switcher.pendingProfile, "")
    wait(switcher.timeoutMs * 2)
    compare(failures.length, 0, "an aborted request must not time out later")
    compare(succeeded.length, 0)
  }
}
