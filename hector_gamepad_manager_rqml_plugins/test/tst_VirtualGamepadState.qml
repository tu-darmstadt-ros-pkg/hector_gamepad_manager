import QtQuick
import QtTest
import "../qml"

TestCase {
  id: testCase
  name: "VirtualGamepadState"

  VirtualGamepadState {
    id: gamepad
  }

  function init() {
    gamepad.step = 0.1
    gamepad.sticky = true
    gamepad.axisResetButtons = []
    gamepad.reset()
  }

  // A released trigger is 0 on the wire, so rest really is all-zero.
  function test_neutral_axes_are_all_zero() {
    var axes = gamepad.joyAxes()
    compare(axes.length, 6)
    for (var i = 0; i < axes.length; ++i) compare(axes[i], 0, "axis " + i + " must rest at 0")
  }

  // Logically a trigger runs 0 to 1, but game_controller_node publishes it negated, so that is
  // what the wire format has to carry for the manager to normalize it back.
  function test_fully_pressed_trigger_goes_out_negated() {
    gamepad.pressKey(Qt.Key_Q, Qt.NoModifier, false)
    gamepad.setAxis("left_trigger", 1.0)
    compare(gamepad.leftTrigger, 1, "held logically as 0 -> 1")
    var axes = gamepad.joyAxes()
    compare(axes[4], -1, "left trigger is axis 4 and goes out as 0 -> -1")
    compare(axes[5], 0)
  }

  // Axis order is SDL's: both sticks first, then the triggers.
  function test_axes_are_in_sdl_order() {
    gamepad.setAxis("left_stick_x", 0.1)
    gamepad.setAxis("left_stick_y", 0.2)
    gamepad.setAxis("right_stick_x", 0.3)
    gamepad.setAxis("right_stick_y", 0.4)
    gamepad.setAxis("left_trigger", 0.5)
    gamepad.setAxis("right_trigger", 0.6)
    var axes = gamepad.joyAxes()
    fuzzyCompare(axes[0], 0.1, 1e-6)
    fuzzyCompare(axes[1], 0.2, 1e-6)
    fuzzyCompare(axes[2], 0.3, 1e-6)
    fuzzyCompare(axes[3], 0.4, 1e-6)
    fuzzyCompare(axes[4], -0.5, 1e-6) // triggers go out negated
    fuzzyCompare(axes[5], -0.6, 1e-6)
  }

  function test_neutral_buttons_are_all_released() {
    var buttons = gamepad.joyButtons()
    compare(buttons.length, 32)
    for (var i = 0; i < buttons.length; ++i) compare(buttons[i], 0)
  }

  function test_left_stick_keys_step_the_axis() {
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    fuzzyCompare(gamepad.leftStickY, 0.1, 1e-6)
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, true) // auto-repeat ramps in sticky mode
    fuzzyCompare(gamepad.leftStickY, 0.2, 1e-6)
    gamepad.pressKey(Qt.Key_S, Qt.NoModifier, false)
    fuzzyCompare(gamepad.leftStickY, 0.1, 1e-6)
  }

  // The two-hand split: the left stick is on WASD, the right stick on IJKL, so both can be driven
  // at once without either hand leaving its cluster.
  function test_both_sticks_are_independent() {
    gamepad.pressKey(Qt.Key_I, Qt.NoModifier, false)
    fuzzyCompare(gamepad.rightStickY, 0.1, 1e-6)
    compare(gamepad.leftStickY, 0)
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    fuzzyCompare(gamepad.leftStickY, 0.1, 1e-6)
    fuzzyCompare(gamepad.rightStickY, 0.1, 1e-6, "the right stick is unaffected by the left")
  }

  // The arrow keys carry the face buttons in the pad's own arrangement: Y on top, A at the bottom,
  // X on the left, B on the right. Getting one of these backwards would be invisible in use until
  // the wrong robot action fired.
  function test_arrow_keys_are_the_face_button_diamond() {
    gamepad.pressKey(Qt.Key_Up, Qt.NoModifier, false)
    verify(gamepad.isButtonPressed("y"), "up must be Y")
    gamepad.pressKey(Qt.Key_Down, Qt.NoModifier, false)
    verify(gamepad.isButtonPressed("a"), "down must be A")
    gamepad.pressKey(Qt.Key_Left, Qt.NoModifier, false)
    verify(gamepad.isButtonPressed("x"), "left must be X")
    gamepad.pressKey(Qt.Key_Right, Qt.NoModifier, false)
    verify(gamepad.isButtonPressed("b"), "right must be B")
    // Nothing on the diamond may disturb a stick.
    compare(gamepad.leftStickX, 0)
    compare(gamepad.rightStickX, 0)
  }

  function test_sticky_axis_survives_key_release() {
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    gamepad.releaseKey(Qt.Key_W, false)
    fuzzyCompare(gamepad.leftStickY, 0.1, 1e-6)
  }

  function test_momentary_axis_recenters_on_release() {
    gamepad.sticky = false
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    compare(gamepad.leftStickY, 1)
    gamepad.releaseKey(Qt.Key_W, false)
    compare(gamepad.leftStickY, 0)
  }

  // Positive is left/up, matching the ROS joy convention the manager reads.
  function test_left_is_positive_on_the_x_axis() {
    gamepad.pressKey(Qt.Key_A, Qt.NoModifier, false)
    verify(gamepad.leftStickX > 0)
    gamepad.resetAxes()
    gamepad.pressKey(Qt.Key_D, Qt.NoModifier, false)
    verify(gamepad.leftStickX < 0)
    gamepad.resetAxes()
    gamepad.pressKey(Qt.Key_J, Qt.NoModifier, false)
    verify(gamepad.rightStickX > 0)
  }

  // SDL reports the d-pad as four real buttons, so it behaves like any other button rather than
  // as a pair of axes that need recentering.
  function test_dpad_is_a_button() {
    gamepad.pressKey(Qt.Key_T, Qt.NoModifier, false)
    verify(gamepad.isButtonPressed("dpad_up"))
    compare(gamepad.joyButtons()[11], 1)
    gamepad.releaseKey(Qt.Key_T, false)
    verify(!gamepad.isButtonPressed("dpad_up"))
  }

  // A trigger only travels one way, but in sticky mode it still has to be walkable back down -
  // otherwise the only way off a held trigger is Space, which zeroes the sticks too.
  function test_triggers_can_be_stepped_back_down() {
    gamepad.pressKey(Qt.Key_Q, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_Q, Qt.NoModifier, false)
    fuzzyCompare(gamepad.leftTrigger, 0.2, 1e-6)
    gamepad.pressKey(Qt.Key_1, Qt.NoModifier, false)
    fuzzyCompare(gamepad.leftTrigger, 0.1, 1e-6)
    gamepad.pressKey(Qt.Key_O, Qt.NoModifier, false)
    fuzzyCompare(gamepad.rightTrigger, 0.1, 1e-6)
    gamepad.pressKey(Qt.Key_9, Qt.NoModifier, false)
    compare(gamepad.rightTrigger, 0)
  }

  // Every axis must be reachable in both directions, or it can be driven somewhere it cannot be
  // brought back from without also zeroing everything else.
  function test_every_axis_can_be_stepped_both_ways() {
    var seen = ({})
    for (var key in gamepad.axisKeys) {
      var axis = gamepad.axisKeys[key]
      if (!seen[axis.axis])
        seen[axis.axis] = ({})
      seen[axis.axis][axis.dir > 0 ? "up" : "down"] = true
    }
    for (var name in seen) {
      verify(seen[name].up, "no key increases '" + name + "'")
      verify(seen[name].down, "no key decreases '" + name + "'")
    }
  }

  function test_axes_clamp_to_their_range() {
    for (var i = 0; i < 30; ++i) gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    compare(gamepad.leftStickY, 1)
    // Triggers only travel one way.
    for (var j = 0; j < 30; ++j) gamepad.setAxis("left_trigger", gamepad.leftTrigger - 0.1)
    compare(gamepad.leftTrigger, 0)
  }

  function test_repeated_steps_land_exactly_on_center() {
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_S, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_S, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_S, Qt.NoModifier, false)
    compare(gamepad.leftStickY, 0)
  }

  function test_modifiers_scale_the_step() {
    gamepad.pressKey(Qt.Key_W, Qt.ShiftModifier, false)
    fuzzyCompare(gamepad.leftStickY, 0.5, 1e-6)
    gamepad.resetAxes()
    gamepad.pressKey(Qt.Key_W, Qt.ControlModifier, false)
    fuzzyCompare(gamepad.leftStickY, 0.02, 1e-6)
  }

  function test_buttons_map_to_their_joy_index() {
    gamepad.pressKey(Qt.Key_Down, Qt.NoModifier, false)
    compare(gamepad.joyButtons()[0], 1)
    gamepad.pressKey(Qt.Key_F4, Qt.NoModifier, false) // share
    compare(gamepad.joyButtons()[15], 1)
    gamepad.releaseKey(Qt.Key_Down, false)
    compare(gamepad.joyButtons()[0], 0)
    compare(gamepad.joyButtons()[15], 1)
  }

  // The four system buttons sit on F1-F4 in the order the controller diagram stacks them, and
  // deliberately not on Enter, Backspace, Home or End: those mean something to a text field, and
  // the panel has one of its own.
  function test_system_buttons_are_the_function_keys() {
    var expected = [{ key: Qt.Key_F1, name: "guide" }, { key: Qt.Key_F2, name: "back" },
                    { key: Qt.Key_F3, name: "start" }, { key: Qt.Key_F4, name: "share" }]
    for (var i = 0; i < expected.length; ++i) {
      gamepad.pressKey(expected[i].key, Qt.NoModifier, false)
      verify(gamepad.isButtonPressed(expected[i].name),
             "F" + (i + 1) + " must press " + expected[i].name)
    }
    var freed = [Qt.Key_Return, Qt.Key_Enter, Qt.Key_Backspace, Qt.Key_Home, Qt.Key_End]
    for (var j = 0; j < freed.length; ++j)
      verify(!gamepad.pressKey(freed[j], Qt.NoModifier, false),
             "text-field keys must not drive the gamepad")
  }

  function test_button_auto_repeat_does_not_retrigger() {
    gamepad.pressKey(Qt.Key_Down, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_Down, Qt.NoModifier, true)
    verify(gamepad.isButtonPressed("a"))
    gamepad.releaseKey(Qt.Key_Down, true) // auto-repeat release must not release the button
    verify(gamepad.isButtonPressed("a"))
    gamepad.releaseKey(Qt.Key_Down, false)
    verify(!gamepad.isButtonPressed("a"))
  }

  function test_escape_stops_everything() {
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_Down, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_Escape, Qt.NoModifier, false)
    compare(gamepad.leftStickY, 0)
    verify(!gamepad.isButtonPressed("a"))
  }

  function test_space_centers_axes_but_keeps_buttons() {
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_Down, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_Space, Qt.NoModifier, false)
    compare(gamepad.leftStickY, 0)
    verify(gamepad.isButtonPressed("a"))
  }

  function test_an_axis_reset_button_centers_every_axis() {
    gamepad.axisResetButtons = ["back"]
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_J, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_O, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_F2, Qt.NoModifier, false) // back
    var axes = gamepad.joyAxes()
    for (var i = 0; i < axes.length; ++i) compare(axes[i], 0, "axis " + i + " must be centered")
    verify(gamepad.isButtonPressed("back"), "the switch press itself must still go out")
  }

  function test_other_buttons_leave_the_axes_alone() {
    gamepad.axisResetButtons = ["back"]
    gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false)
    gamepad.pressKey(Qt.Key_F3, Qt.NoModifier, false) // start
    fuzzyCompare(gamepad.leftStickY, 0.1, 1e-6)
  }

  function test_unknown_keys_are_not_consumed() {
    verify(!gamepad.pressKey(Qt.Key_F5, Qt.NoModifier, false))
    verify(!gamepad.releaseKey(Qt.Key_F5, false))
    verify(gamepad.pressKey(Qt.Key_W, Qt.NoModifier, false))
  }

  // isActive() has to agree with the manager about when an axis becomes a virtual button,
  // otherwise the bindings table highlights the wrong row.
  function test_virtual_button_activates_at_the_managers_deadzone() {
    gamepad.setAxis("left_stick_y", 0.5)
    verify(!gamepad.isActive("left_stick_up"), "0.5 is not yet past the deadzone")
    gamepad.setAxis("left_stick_y", 0.6)
    verify(gamepad.isActive("left_stick_up"))
    verify(!gamepad.isActive("left_stick_down"))
  }

  // Guards against a future rebind quietly stealing a key that already drives something else.
  function test_no_key_drives_both_an_axis_and_a_button() {
    for (var key in gamepad.axisKeys)
      verify(gamepad.buttonKeys[key] === undefined,
             "key code " + key + " is bound to both an axis and a button")
  }

  function test_every_bound_input_has_a_key_label() {
    for (var axisKey in gamepad.axisKeys) {
      var axis = gamepad.axisKeys[axisKey]
      verify(gamepad.keyLabels[axis.axis] !== undefined,
             "axis '" + axis.axis + "' is bound but missing from keyLabels")
    }
    for (var buttonKey in gamepad.buttonKeys) {
      var name = gamepad.buttonKeys[buttonKey]
      verify(gamepad.keyLabels[name] !== undefined,
             "button '" + name + "' is bound but missing from keyLabels")
    }
  }

  function test_every_key_label_names_a_real_input() {
    for (var name in gamepad.keyLabels) {
      verify(gamepad.buttonIndices[name] !== undefined
             || gamepad.virtualButtonSources[name] !== undefined
             || gamepad.axisValue(name) !== undefined,
             "keyLabels entry '" + name + "' is not a known input")
    }
  }
}
