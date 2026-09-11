import QtQuick
import QtTest
import "../qml"

// The dot moves like the physical stick: positive x is left, positive y is up (ROS joy convention).
TestCase {
  id: testCase
  name: "StickPad"
  when: windowShown
  width: 200
  height: 200

  // Both are rebuilt in init(), so no deflection carries over between tests.
  readonly property var pad: padLoader.item
  readonly property var trigger: triggerLoader.item

  ItemFinder {
    id: finder
  }

  Loader {
    id: padLoader
    sourceComponent: Component {
      StickPad {
        width: 80
      }
    }
  }

  Loader {
    id: triggerLoader
    sourceComponent: Component {
      TriggerBar {}
    }
  }

  function init() {
    padLoader.active = false
    padLoader.active = true
    triggerLoader.active = false
    triggerLoader.active = true
  }

  //! The dot has no objectName; it is the only child of its size with a radius.
  function dot() {
    return finder.first(pad, function (child) {
      return child.radius !== undefined && child.width === 9
    })
  }

  function test_centered_dot_sits_in_the_middle() {
    var d = dot()
    fuzzyCompare(d.x, (80 - 9) / 2, 0.5)
    fuzzyCompare(d.y, (80 - 9) / 2, 0.5)
  }

  function test_positive_x_moves_the_dot_left() {
    var center = dot().x
    pad.xValue = 1
    verify(dot().x < center, "positive x is left, so the dot must move left")
    pad.xValue = -1
    verify(dot().x > center)
  }

  function test_positive_y_moves_the_dot_up() {
    var center = dot().y
    pad.yValue = 1
    verify(dot().y < center, "positive y is up, so the dot must move up the screen")
    pad.yValue = -1
    verify(dot().y > center)
  }

  function test_dot_stays_inside_the_field_at_full_deflection() {
    pad.xValue = 1
    pad.yValue = 1
    verify(dot().x >= 0)
    verify(dot().y >= 0)
    pad.xValue = -1
    pad.yValue = -1
    verify(dot().x + dot().width <= pad.width)
    verify(dot().y + dot().height <= pad.width)
  }

  function test_any_deflection_highlights() {
    verify(!pad.deflected, "a centered stick is not deflected")
    pad.yValue = 0.1
    verify(pad.deflected)
    pad.yValue = 0
    pad.xValue = -0.1
    verify(pad.deflected)
  }

  function test_any_trigger_travel_highlights() {
    verify(!trigger.pressed, "a released trigger is not pressed")
    trigger.value = 0.1
    verify(trigger.pressed)
  }
}
