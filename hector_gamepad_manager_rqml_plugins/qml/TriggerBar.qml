import QtQuick
import QtQuick.Controls
import RQml.Elements

// Trigger readout: a vertical bar filling upward from 0 to 1, with its name and key underneath.
Item {
  id: bar

  //! Trigger travel from 0 (released) to 1 (fully pressed), not the negated wire value.
  property real value: 0

  //! Name shown under the bar, e.g. "LT".
  property string label: ""

  //! Key that drives it, shown under the name.
  property string keyHint: ""

  //! Height of the bar, e.g. a StickPad's width so the two line up.
  property real trackHeight: 100

  property color contentColor: palette.text
  property color activeColor: palette.highlight

  //! The trigger is pulled; the bar is then drawn in activeColor.
  readonly property bool pressed: value > 0

  implicitWidth: Math.max(track.width, caption.implicitWidth)
  implicitHeight: trackHeight + caption.implicitHeight + 2

  Rectangle {
    id: track
    anchors.horizontalCenter: parent.horizontalCenter
    width: Math.round(bar.trackHeight / 6)
    height: bar.trackHeight
    radius: 2
    color: "transparent"
    border.width: 1
    border.color: bar.pressed ? bar.activeColor : bar.contentColor

    Rectangle {
      anchors.left: parent.left
      anchors.right: parent.right
      anchors.bottom: parent.bottom
      anchors.margins: 1
      height: Math.max(0, (parent.height - 2) * Math.max(0, Math.min(1, bar.value)))
      radius: 1
      color: bar.pressed ? bar.activeColor : bar.contentColor
      opacity: bar.pressed ? 1 : 0.5
    }
  }

  Column {
    id: caption
    anchors.top: track.bottom
    anchors.topMargin: 2
    anchors.horizontalCenter: parent.horizontalCenter
    spacing: 0

    Caption {
      anchors.horizontalCenter: parent.horizontalCenter
      text: bar.label
    }
    Label {
      anchors.horizontalCenter: parent.horizontalCenter
      font.bold: true
      visible: bar.keyHint !== ""
      color: bar.pressed ? bar.activeColor : bar.contentColor
      text: bar.keyHint
    }
  }
}
