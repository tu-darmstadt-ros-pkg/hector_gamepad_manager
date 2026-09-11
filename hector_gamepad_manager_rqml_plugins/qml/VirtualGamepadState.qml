import QtQuick

// Keyboard-driven gamepad state, and its translation to the sensor_msgs/Joy wire format.
//
// Deliberately free of ROS and of any UI so the conversion can be unit-tested on its own: it holds
// *logical* input values (sticks -1..1, triggers 0..1) and only converts to wire values in
// joyAxes() / joyButtons().
//
// The wire layout is SDL's canonical GameController layout, the same one game_controller_node
// publishes and hector_gamepad_manager's gamepad_buttons.hpp reads. Being the producer side, this
// file has to know that layout; consumers of the published mapping key off the names instead.
QtObject {
  id: state

  //! Amount an axis moves per key press in sticky mode. Shift/Ctrl scale it, see stepFor().
  property real step: 0.1

  //! Sticky (throttle-like) axes: a key press moves the axis and it stays there. When false the
  //! axes are momentary: held = fully deflected, released = centered.
  property bool sticky: true

  // Logical axis values. Sticks are -1..1 with positive meaning left/up (the ROS joy convention
  // the manager expects); triggers are 0..1 with 0 = released. The d-pad is not here: SDL reports
  // it as four real buttons.
  property real leftStickX: 0
  property real leftStickY: 0
  property real leftTrigger: 0
  property real rightStickX: 0
  property real rightStickY: 0
  property real rightTrigger: 0

  //! Pressed physical buttons as { canonical name: true }. Replaced wholesale on every change so
  //! bindings on it re-evaluate.
  property var buttonStates: ({})

  //! Buttons whose press centers every axis, e.g. the config switches.
  property var axisResetButtons: []

  //! Mirrors HectorGamepadManager::AXIS_DEADZONE, so the UI marks an axis as triggering its
  //! virtual button at the same point the manager does.
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

  //! Number of buttons published. The manager reads ids 0-31 straight off the wire; sending the
  //! full range covers the paddles and touchpad some pads report.
  readonly property int buttonCount: 32

  // Two-hand layout that mirrors the pad's own geometry:
  //
  //   Q W E        shoulders           U I O        shoulders        ↑          face buttons
  //    A S D       left stick           J K L       right stick    ← ↓ →       (same diamond
  //                                                                             as the pad)
  //   T F G H      d-pad                C / V       stick clicks
  //
  //   F1 F2 F3 F4  guide, back, start, share - the four system buttons, in the order the
  //                controller diagram stacks them
  //
  // Two diamonds do the heavy lifting. The right stick is IJKL, and the face buttons are the
  // arrow keys - whose diamond has exactly the pad's own arrangement, Y on top, A at the bottom,
  // X left and B right - so neither needs memorising. That frees the left hand for WASD.
  //
  // What you can hold at once: left stick + right stick, and left stick + face buttons. Right
  // stick and face buttons share the right hand, which is the combination you rarely need.
  //
  // Every key is in the same physical place on QWERTZ as on QWERTY: no punctuation keys, no
  // numpad, and nothing that depends on where Y and Z sit.

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
    // Triggers sit outboard of each stick's top row, bumpers inboard, mirrored on both hands.
    // A trigger only travels one way, but in sticky mode it still needs a way back: the key
    // directly above each one eases it off, so a trigger can be trimmed without Space zeroing
    // the sticks along with it. (1 sits above Q and 9 above O on QWERTY and QWERTZ alike.)
    m[Qt.Key_Q] = ({ axis: "left_trigger", dir: 1 })
    m[Qt.Key_1] = ({ axis: "left_trigger", dir: -1 })
    m[Qt.Key_O] = ({ axis: "right_trigger", dir: 1 })
    m[Qt.Key_9] = ({ axis: "right_trigger", dir: -1 })
    return m
  }

  // Key code -> physical button name. The arrow keys carry the face buttons in the pad's own
  // arrangement, so their positions match what is printed on the controller.
  //
  // The four system buttons sit on F1-F4, in the order the diagram stacks them. They are rarely
  // pressed, so being away from the hands costs nothing, and unlike Enter, Backspace, Home and End
  // they mean nothing to a text field, which the panel has one of.
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

  //! Canonical input name -> the key(s) that drive it, for the bindings table. Axis-derived
  //! virtual buttons list the single key that pushes the axis that way.
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

  // Virtual axis button name -> the axis it reads and the sign that activates it. Mirrors the
  // assignment order of convertJoyToGamepadInputs().
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

  //! Triggers only travel in one direction, so they clamp to [0, 1] rather than [-1, 1].
  function axisMinimum(name) {
    return (name === "left_trigger" || name === "right_trigger") ? 0 : -1
  }

  function setAxis(name, value) {
    var clamped = Math.max(axisMinimum(name), Math.min(1, value))
    // Snap to zero so repeated 0.1 steps land exactly on center instead of 5.55e-17.
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

  //! True if the input is currently deflected/pressed far enough for the manager to act on it.
  //! Covers physical buttons, virtual axis buttons and raw axes alike.
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

  //! Full stop: everything back to rest.
  function reset() {
    resetAxes()
    buttonStates = ({})
  }

  //! Handle a key press. Returns true if the key belongs to the gamepad and was consumed.
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
        // Auto-repeat is what makes a held key ramp the axis, so it is not filtered here.
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

  //! Axis values in Joy wire format, in SDL GameController order. Triggers are held logically as
  //! 0 (released) to 1 (pressed) but go out negated, the way game_controller_node publishes them;
  //! the manager flips them back. See isTriggerAxis() in gamepad_buttons.hpp.
  function joyAxes() {
    return [leftStickX, leftStickY, rightStickX, rightStickY, -leftTrigger, -rightTrigger]
  }

  //! Button values in Joy wire format. Only physical buttons go on the wire; the manager derives
  //! the virtual axis buttons from the axes itself.
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
