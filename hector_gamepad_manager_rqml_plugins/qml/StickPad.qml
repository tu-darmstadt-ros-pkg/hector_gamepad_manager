import QtQuick
import QtQuick.Controls

// Two-axis readout drawn the way the stick actually moves: a dot inside a square, with the
// manager's deadzone marked so you can see the exact point an axis starts acting as a virtual
// button. Read-only: the keyboard drives the gamepad, and a draggable stick would be a way around
// the panel's capture rules.
//
// Positive x is left and positive y is up, matching the ROS joy convention, so the dot moves the
// way the physical stick would.
Item {
  id: pad

  //! Axis values, -1..1 each.
  property real xValue: 0
  property real yValue: 0

  //! Deflection past which the manager treats the axis as a pressed virtual button.
  property real deadzone: 0.5

  //! Draw the dot only at the nine positions a d-pad can report, and skip the deadzone ring.
  property bool discrete: false

  //! Caption under the pad, e.g. "Left stick".
  property string label: ""

  //! Keys that drive the axes, shown beside the caption.
  property string keyHint: ""

  property color contentColor: palette.text
  property color activeColor: palette.highlight

  readonly property bool deflected: Math.abs(xValue) > deadzone || Math.abs(yValue) > deadzone

  implicitWidth: 72
  implicitHeight: 72 + caption.implicitHeight + 2

  Rectangle {
    id: field
    width: pad.width
    height: pad.width
    radius: 3
    color: "transparent"
    border.width: 1
    border.color: pad.deflected ? pad.activeColor : pad.contentColor

    // Deadzone ring: inside it the manager sees no virtual button press.
    Rectangle {
      visible: !pad.discrete
      anchors.centerIn: parent
      width: field.width * pad.deadzone
      height: width
      radius: width / 2
      color: "transparent"
      border.width: 1
      border.color: pad.contentColor
      opacity: 0.35
    }

    // Cross-hair through the centre, so a small deflection is still visible against something.
    Rectangle {
      anchors.centerIn: parent
      width: parent.width - 8
      height: 1
      color: pad.contentColor
      opacity: 0.2
    }
    Rectangle {
      anchors.centerIn: parent
      width: 1
      height: parent.height - 8
      color: pad.contentColor
      opacity: 0.2
    }

    Rectangle {
      id: dot
      width: 9
      height: 9
      radius: width / 2
      color: pad.deflected ? pad.activeColor : pad.contentColor
      // Positive x is left, so it subtracts; positive y is up, so it subtracts too.
      x: (field.width - width) / 2 - pad.xValue * (field.width - width) / 2
      y: (field.height - height) / 2 - pad.yValue * (field.height - height) / 2
    }
  }

  Column {
    id: caption
    anchors.top: field.bottom
    anchors.topMargin: 2
    width: parent.width
    spacing: 0

    Label {
      width: parent.width
      horizontalAlignment: Text.AlignHCenter
      font.pixelSize: 11
      elide: Text.ElideRight
      text: pad.label
    }
    Label {
      width: parent.width
      horizontalAlignment: Text.AlignHCenter
      font.pixelSize: 11
      font.bold: true
      elide: Text.ElideRight
      visible: pad.keyHint !== ""
      color: pad.deflected ? pad.activeColor : pad.contentColor
      text: pad.keyHint
    }
  }
}
