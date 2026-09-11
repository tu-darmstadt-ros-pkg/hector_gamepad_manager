import QtQuick

// Tracks a config-switch request until joy_teleop_profile confirms it or it times out.
QtObject {
  id: switcher

  //! Config switches from the mapping, as [{ config: string, name: string }].
  property var configSwitches: []

  //! Canonical button name -> Joy index. Names absent from it cannot be pressed on the wire.
  property var pressableButtons: ({})

  //! Profile the robot currently reports.
  property string activeProfile: ""

  //! Requested but unconfirmed profile, "" when idle.
  property string pendingProfile: ""

  //! How long to wait for the acknowledgement before giving up.
  property int timeoutMs: 2000

  readonly property bool busy: pendingProfile !== ""

  //! Canonical names of the switch buttons that lead away from the active profile.
  readonly property var switchAwayButtons: configSwitches.filter(function (configSwitch) {
    return configSwitch.config !== activeProfile
  }).map(function (configSwitch) {
    return configSwitch.name
  })

  //! The caller should press this button.
  signal pressRequested(string buttonName)

  //! The robot confirmed the requested profile.
  signal succeeded(string config)

  //! reason: "unbound" (no switch button), "unpressable" (bound to a virtual axis button) or
  //! "timeout". detail: the button name, or the still active profile on timeout.
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

  //! Ask for `config`. Returns whether a press was emitted. Ignored while busy or when `config` is
  //! already active; any other refusal also emits failed().
  function request(config) {
    if (busy || config === "" || config === activeProfile)
      return false
    var name = switchButtonFor(config)
    if (name === "") {
      failed(config, "unbound", "")
      return false
    }
    // Virtual axis buttons can be config switches too, but they are not on the wire.
    if (pressableButtons[name] === undefined) {
      failed(config, "unpressable", name)
      return false
    }
    pendingProfile = config
    timeoutTimer.restart()
    pressRequested(name)
    return true
  }

  //! Drop an in-flight request without reporting an outcome.
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
