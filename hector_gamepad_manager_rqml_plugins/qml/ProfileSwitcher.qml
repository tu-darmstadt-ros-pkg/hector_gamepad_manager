import QtQuick

// Tracks a config-switch request from the press that starts it to the manager's acknowledgement.
//
// The manager republishes joy_teleop_profile on every switch, so that message is the only
// confirmation there is - and the only way to notice a switch someone made on a real gamepad.
// Assign activeProfile from it and this watches for its own request to land, or gives up.
//
// Deliberately free of ROS and of any UI so the state machine can be tested on its own: the
// caller performs the button press and phrases the messages.
QtObject {
  id: switcher

  //! Config switches from the mapping, as [{ config: string, name: string }].
  property var configSwitches: []

  //! Canonical button name -> Joy index. Names absent from it cannot be pressed on the wire.
  property var pressableButtons: ({})

  //! Profile the robot currently reports. Assign it; changes drive the confirmation.
  property string activeProfile: ""

  //! Requested but unconfirmed profile, "" when idle.
  property string pendingProfile: ""

  //! How long to wait for the acknowledgement before giving up.
  property int timeoutMs: 2000

  readonly property bool busy: pendingProfile !== ""

  //! The caller should press this button as if it came from the gamepad.
  signal pressRequested(string buttonName)

  //! The robot confirmed the requested profile.
  signal succeeded(string config)

  //! reason is "unbound" (no switch button maps to it), "unpressable" (it maps to a virtual axis
  //! button, which is not on the wire) or "timeout". detail carries the button name where useful.
  signal failed(string config, string reason, string detail)

  property Timer timeoutTimer: Timer {
    interval: switcher.timeoutMs
    onTriggered: {
      var requested = switcher.pendingProfile
      switcher.pendingProfile = ""
      switcher.failed(requested, "timeout", switcher.activeProfile)
    }
  }

  //! Canonical name of the button that switches to `config`, or "" if none does.
  function switchButtonFor(config) {
    for (var i = 0; i < configSwitches.length; ++i)
      if (configSwitches[i].config === config)
        return configSwitches[i].name || ""
    return ""
  }

  //! Ask for `config`. Returns true if a press was emitted, false if it failed outright - in which
  //! case failed() has already fired. Requests are ignored while one is in flight or when the
  //! robot is already in that config.
  function request(config) {
    if (busy || config === "" || config === activeProfile)
      return false
    var name = switchButtonFor(config)
    if (name === "") {
      failed(config, "unbound", "")
      return false
    }
    // Config switches are indexed over the virtual axis buttons too, and those are derived by the
    // manager from the axes rather than read off the wire, so there is no button to press.
    if (pressableButtons[name] === undefined) {
      failed(config, "unpressable", name)
      return false
    }
    pendingProfile = config
    timeoutTimer.restart()
    pressRequested(name)
    return true
  }

  //! Drop an in-flight request without reporting either outcome, e.g. when publishing stops and
  //! the press can no longer reach the robot.
  function abort() {
    timeoutTimer.stop()
    pendingProfile = ""
  }

  onActiveProfileChanged: {
    if (pendingProfile === "" || activeProfile !== pendingProfile)
      return
    var confirmed = pendingProfile
    pendingProfile = ""
    timeoutTimer.stop()
    succeeded(confirmed)
  }
}
