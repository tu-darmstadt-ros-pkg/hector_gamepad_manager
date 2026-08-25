import QtQuick
import QtQuick.Controls

// One-way readout for a trigger: a bar filling 0 -> 1, with the manager's deadzone marked so you
// can see where it starts counting as a pressed virtual button.
Item {
  id: bar

  //! Trigger travel, 0 (released) to 1 (fully pressed). This is the logical value, not the
  //! inverted one that goes on the wire.
  property real value: 0

  //! Travel past which the manager treats the trigger as a pressed virtual button.
  property real deadzone: 0.5

  //! Name shown left of the bar, e.g. "LT".
  property string label: ""

  //! Key that drives it, shown right of the bar.
  property string keyHint: ""

  property color contentColor: palette.text
  property color activeColor: palette.highlight

  readonly property bool pressed: value > deadzone

  implicitWidth: 130
  implicitHeight: Math.max(track.implicitHeight, name.implicitHeight)

  Label {
    id: name
    anchors.left: parent.left
    anchors.verticalCenter: parent.verticalCenter
    width: 18
    font.pixelSize: 11
    font.bold: true
    color: bar.pressed ? bar.activeColor : bar.contentColor
    text: bar.label
  }

  Rectangle {
    id: track
    anchors.left: name.right
    anchors.right: hint.left
    anchors.rightMargin: 4
    anchors.verticalCenter: parent.verticalCenter
    implicitHeight: 10
    height: 10
    radius: 2
    color: "transparent"
    border.width: 1
    border.color: bar.pressed ? bar.activeColor : bar.contentColor

    Rectangle {
      anchors.left: parent.left
      anchors.top: parent.top
      anchors.bottom: parent.bottom
      anchors.margins: 1
      width: Math.max(0, (parent.width - 2) * Math.max(0, Math.min(1, bar.value)))
      radius: 1
      color: bar.pressed ? bar.activeColor : bar.contentColor
      opacity: bar.pressed ? 1 : 0.5
    }

    // Deadzone mark.
    Rectangle {
      x: parent.width * bar.deadzone
      width: 1
      height: parent.height
      color: bar.contentColor
      opacity: 0.4
    }
  }

  Label {
    id: hint
    anchors.right: parent.right
    anchors.verticalCenter: parent.verticalCenter
    horizontalAlignment: Text.AlignRight
    font.pixelSize: 11
    font.bold: true
    color: bar.pressed ? bar.activeColor : bar.contentColor
    text: bar.keyHint
  }
}
