import QtQuick
import QtQuick.Controls
import RQml.Elements

// Two-axis readout: a dot in a square. Positive x is left and positive y is up (ROS joy
// convention).
Item {
  id: pad

  //! Axis values, -1..1 each.
  property real xValue: 0
  property real yValue: 0

  //! Caption under the pad, e.g. "Left stick".
  property string label: ""

  //! Keys that drive the axes, shown under the caption.
  property string keyHint: ""

  property color contentColor: palette.text
  property color activeColor: palette.highlight

  //! The stick is off center; the pad is then drawn in activeColor.
  readonly property bool deflected: xValue !== 0 || yValue !== 0

  // Sized from the key hint's font, so it grows with the text.
  implicitWidth: Math.round(keyText.implicitHeight * 6)
  implicitHeight: width + caption.implicitHeight + 2

  Rectangle {
    id: field
    width: pad.width
    height: pad.width
    radius: 3
    color: "transparent"
    border.width: 1
    border.color: pad.deflected ? pad.activeColor : pad.contentColor

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
      width: Math.round(pad.width * 0.11)
      height: width
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
    Label {
      id: keyText
      width: parent.width
      horizontalAlignment: Text.AlignHCenter
      font.bold: true
      elide: Text.ElideRight
      visible: pad.keyHint !== ""
      color: pad.deflected ? pad.activeColor : pad.contentColor
      text: pad.keyHint
    }
  }
}
