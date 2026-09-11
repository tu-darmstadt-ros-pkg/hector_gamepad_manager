import QtQuick
import RQml.Elements

// Two-axis readout: a dot in a square with the manager's deadzone marked. Positive x is left and
// positive y is up (ROS joy convention).
Item {
  id: pad

  //! Axis values, -1..1 each.
  property real xValue: 0
  property real yValue: 0

  //! Deflection past which the manager treats the axis as a pressed virtual button.
  property real deadzone: 0.5

  //! Hide the deadzone ring, for inputs like the d-pad that have none.
  property bool discrete: false

  //! Caption under the pad, e.g. "Left stick".
  property string label: ""

  //! Keys that drive the axes, shown under the caption.
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

    // Deadzone ring.
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

    // Cross-hair.
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
      // Positive x is left and positive y is up, so both subtract.
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

    Caption {
      width: parent.width
      horizontalAlignment: Text.AlignHCenter
      text: pad.label
    }
    Caption {
      width: parent.width
      horizontalAlignment: Text.AlignHCenter
      font.bold: true
      visible: pad.keyHint !== ""
      color: pad.deflected ? pad.activeColor : pad.contentColor
      text: pad.keyHint
    }
  }
}
