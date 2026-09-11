import QtQuick

// Keyboard-driven gamepad state. joyAxes() and joyButtons() convert it to sensor_msgs/Joy in SDL's
// GameController layout, the one game_controller_node publishes.
QtObject {
  id: state

  //! Amount an axis moves per key press in sticky mode. Shift/Ctrl scale it, see stepFor().
  property real step: 0.1

  //! Axes keep their value between key presses. Otherwise a held key deflects fully.
  property bool sticky: true

  // Sticks are -1..1, positive left/up (ROS joy convention). Triggers are 0..1, 0 is released.
  // The d-pad is four buttons.
  property real leftStickX: 0
  property real leftStickY: 0
  property real leftTrigger: 0
  property real rightStickX: 0
  property real rightStickY: 0
  property real rightTrigger: 0

  //! Pressed buttons as { name: true }. Reassigned on every change so bindings re-evaluate.
  property var buttonStates: ({})

  //! Buttons whose press centers every axis, e.g. the config switches.
  property var axisResetButtons: []

  //! Mirrors HectorGamepadManager::AXIS_DEADZONE.
  readonly property real axisDeadzone: 0.5

  //! Physical button name -> Joy button index, in SDL GameController order.
  readonly property var buttonIndices: ({
    "a": 0, "b": 1, "x": 2, "y": 3,
    "back": 4, "guide": 5, "start": 6,
    "left_stick_click": 7, "right_stick_click": 8,
    "left_bumper": 9, "right_bumper": 10,
    "dpad_up": 11, "dpad_down": 12, "dpad_left": 13, "dpad_right": 14,
    "share": 15
  })

  //! Buttons published; the manager reads ids 0-31.
  readonly property int buttonCount: 32

  // Keyboard layout, mirroring the pad:
  //
  //   Q W E        shoulders           U I O        shoulders        ↑          face buttons
  //    A S D       left stick           J K L       right stick    ← ↓ →
  //
  //   T F G H      d-pad                C / V       stick clicks
  //
  //   F1 F2 F3 F4  guide, back, start, share
  //
  // Every key sits in the same place on QWERTY and QWERTZ.

  // Key code -> axis descriptor. `dir` is the sign a press applies.
  readonly property var axisKeys: {
    var m = ({})
    m[Qt.Key_A] = ({ axis: "left_stick_x", dir: 1 })
    m[Qt.Key_D] = ({ axis: "left_stick_x", dir: -1 })
    m[Qt.Key_W] = ({ axis: "left_stick_y", dir: 1 })
    m[Qt.Key_S] = ({ axis: "left_stick_y", dir: -1 })
    m[Qt.Key_J] = ({ axis: "right_stick_x", dir: 1 })
    m[Qt.Key_L] = ({ axis: "right_stick_x", dir: -1 })
    m[Qt.Key_I] = ({ axis: "right_stick_y", dir: 1 })
    m[Qt.Key_K] = ({ axis: "right_stick_y", dir: -1 })
    // 1 and 9 step the triggers back down, so they can be eased off without Space.
    m[Qt.Key_Q] = ({ axis: "left_trigger", dir: 1 })
    m[Qt.Key_1] = ({ axis: "left_trigger", dir: -1 })
    m[Qt.Key_O] = ({ axis: "right_trigger", dir: 1 })
    m[Qt.Key_9] = ({ axis: "right_trigger", dir: -1 })
    return m
  }

  // Key code -> physical button name. The arrow keys form the face-button diamond. The system
  // buttons use F1-F4, since the text fields need Enter, Backspace, Home and End.
  readonly property var buttonKeys: {
    var m = ({})
    m[Qt.Key_Up] = "y"
    m[Qt.Key_Down] = "a"
    m[Qt.Key_Left] = "x"
    m[Qt.Key_Right] = "b"
    m[Qt.Key_E] = "left_bumper"
    m[Qt.Key_U] = "right_bumper"
    m[Qt.Key_F1] = "guide"
    m[Qt.Key_F2] = "back"
    m[Qt.Key_F3] = "start"
    m[Qt.Key_F4] = "share"
    m[Qt.Key_C] = "left_stick_click"
    m[Qt.Key_V] = "right_stick_click"
    m[Qt.Key_T] = "dpad_up"
    m[Qt.Key_G] = "dpad_down"
    m[Qt.Key_F] = "dpad_left"
    m[Qt.Key_H] = "dpad_right"
    return m
  }

  //! Canonical input name -> the key(s) that drive it, for the bindings table.
  readonly property var keyLabels: ({
    "left_stick_x": "A / D", "left_stick_y": "W / S",
    "left_stick_left": "A", "left_stick_right": "D",
    "left_stick_up": "W", "left_stick_down": "S",
    "right_stick_x": "J / L", "right_stick_y": "I / K",
    "right_stick_left": "J", "right_stick_right": "L",
    "right_stick_up": "I", "right_stick_down": "K",
    "left_trigger": "Q / 1", "right_trigger": "O / 9",
    "left_bumper": "E", "right_bumper": "U",
    "dpad_left": "F", "dpad_right": "H", "dpad_up": "T", "dpad_down": "G",
    "a": "↓", "b": "→", "x": "←", "y": "↑",
    "guide": "F1", "back": "F2", "start": "F3", "share": "F4",
    "left_stick_click": "C", "right_stick_click": "V"
  })

  // Virtual axis button -> source axis and activating sign, as in convertJoyToGamepadInputs().
  readonly property var virtualButtonSources: ({
    "left_stick_left": ({ axis: "left_stick_x", dir: 1 }),
    "left_stick_right": ({ axis: "left_stick_x", dir: -1 }),
    "left_stick_up": ({ axis: "left_stick_y", dir: 1 }),
    "left_stick_down": ({ axis: "left_stick_y", dir: -1 }),
    "left_trigger": ({ axis: "left_trigger", dir: 1 }),
    "right_stick_left": ({ axis: "right_stick_x", dir: 1 }),
    "right_stick_right": ({ axis: "right_stick_x", dir: -1 }),
    "right_stick_up": ({ axis: "right_stick_y", dir: 1 }),
    "right_stick_down": ({ axis: "right_stick_y", dir: -1 }),
    "right_trigger": ({ axis: "right_trigger", dir: 1 })
  })

  function axisValue(name) {
    switch (name) {
    case "left_stick_x": return leftStickX
    case "left_stick_y": return leftStickY
    case "left_trigger": return leftTrigger
    case "right_stick_x": return rightStickX
    case "right_stick_y": return rightStickY
    case "right_trigger": return rightTrigger
    }
    return 0
  }

  //! Lower bound: 0 for triggers, -1 for sticks.
  function axisMinimum(name) {
    return (name === "left_trigger" || name === "right_trigger") ? 0 : -1
  }

  function setAxis(name, value) {
    var clamped = Math.max(axisMinimum(name), Math.min(1, value))
    // Snap float residue from repeated 0.1 steps (e.g. 5.55e-17) to exactly 0.
    if (Math.abs(clamped) < 1e-6)
      clamped = 0
    switch (name) {
    case "left_stick_x": leftStickX = clamped; break
    case "left_stick_y": leftStickY = clamped; break
    case "left_trigger": leftTrigger = clamped; break
    case "right_stick_x": rightStickX = clamped; break
    case "right_stick_y": rightStickY = clamped; break
    case "right_trigger": rightTrigger = clamped; break
    }
  }

  function setButton(name, pressed) {
    if (pressed && axisResetButtons.indexOf(name) >= 0)
      resetAxes()
    var next = ({})
    for (var key in buttonStates) next[key] = buttonStates[key]
    if (pressed)
      next[name] = true
    else
      delete next[name]
    buttonStates = next
  }

  function isButtonPressed(name) { return buttonStates[name] === true }

  //! The input is pressed or deflected far enough for the manager to act on it.
  function isActive(name) {
    if (buttonIndices[name] !== undefined)
      return isButtonPressed(name)
    var source = virtualButtonSources[name]
    if (source)
      return axisValue(source.axis) * source.dir > axisDeadzone
    return Math.abs(axisValue(name)) > 1e-6
  }

  function stepFor(modifiers) {
    if (modifiers & Qt.ShiftModifier)
      return step * 5
    if (modifiers & Qt.ControlModifier)
      return step * 0.2
    return step
  }

  //! Center every axis, leaving buttons alone.
  function resetAxes() {
    leftStickX = 0
    leftStickY = 0
    leftTrigger = 0
    rightStickX = 0
    rightStickY = 0
    rightTrigger = 0
  }

  //! Release everything.
  function reset() {
    resetAxes()
    buttonStates = ({})
  }

  //! Handle a key press. Returns true if the key belongs to the gamepad.
  function pressKey(key, modifiers, autoRepeat) {
    if (key === Qt.Key_Escape) {
      reset()
      return true
    }
    if (key === Qt.Key_Space) {
      resetAxes()
      return true
    }
    var axis = axisKeys[key]
    if (axis) {
      if (sticky) {
        // Auto-repeat is what ramps the axis while a key is held.
        setAxis(axis.axis, axisValue(axis.axis) + axis.dir * stepFor(modifiers))
      } else if (!autoRepeat) {
        setAxis(axis.axis, axis.dir)
      }
      return true
    }
    var button = buttonKeys[key]
    if (button) {
      if (!autoRepeat)
        setButton(button, true)
      return true
    }
    return false
  }

  //! Handle a key release. Returns true if the key belongs to the gamepad.
  function releaseKey(key, autoRepeat) {
    if (autoRepeat)
      return axisKeys[key] !== undefined || buttonKeys[key] !== undefined
    var axis = axisKeys[key]
    if (axis) {
      if (!sticky)
        setAxis(axis.axis, 0)
      return true
    }
    var button = buttonKeys[key]
    if (button) {
      setButton(button, false)
      return true
    }
    return false
  }

  //! Axes in Joy wire order. Triggers go out negated like game_controller_node publishes them, see
  //! isTriggerAxis() in gamepad_buttons.hpp.
  function joyAxes() {
    return [leftStickX, leftStickY, rightStickX, rightStickY, -leftTrigger, -rightTrigger]
  }

  //! Physical buttons in Joy wire order. The manager derives virtual axis buttons from the axes.
  function joyButtons() {
    var buttons = new Array(buttonCount).fill(0)
    for (var name in buttonStates) {
      var index = buttonIndices[name]
      if (buttonStates[name] && index !== undefined)
        buttons[index] = 1
    }
    return buttons
  }
}
