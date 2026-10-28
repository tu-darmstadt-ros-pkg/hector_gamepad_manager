import QtQuick
import QtQuick.Controls.Material
import Qt5Compat.GraphicalEffects

// Tinted glyph for one gamepad control. Icons by Zacksly (CC BY 3.0), see svgs/ATTRIBUTION.md.
Item {
  id: icon

  //! Control key, one of the keys in glyphFiles. Unknown keys render nothing.
  property string controlKey

  //! Tint for the monochrome (non-face) glyphs.
  property color contentColor: palette.text

  //! Tint for a reserved control.
  property color accentColor: palette.highlight

  //! The control is a config-switch button.
  property bool reserved: false

  //! Tint while the control is pressed or deflected.
  property color activeColor: palette.highlight

  //! The control is pressed or deflected.
  property bool active: false

  //! Edge length of the square glyph.
  property real size: 20

  //! A, B, X and Y keep the colours of the physical pad.
  readonly property var faceColors: ({
    "a": Material.color(Material.Green, Material.Shade700),
    "b": Material.color(Material.Red, Material.Shade700),
    "x": Material.color(Material.Blue, Material.Shade700),
    "y": Material.color(Material.Amber, Material.Shade700)
  })
  readonly property var glyphFiles: ({
    "a": "A.svg", "b": "B.svg", "x": "X.svg", "y": "Y.svg",
    "lb": "Left Bumper.svg", "rb": "Right Bumper.svg",
    "lt": "Left Trigger.svg", "rt": "Right Trigger.svg",
    "lstick": "Left Stick.svg", "rstick": "Right Stick.svg",
    "lstick_click": "Left Stick Click.svg", "rstick_click": "Right Stick Click.svg",
    "dpad": "D-Pad.svg", "dpad_up": "D-Pad Up.svg", "dpad_down": "D-Pad Down.svg",
    "dpad_left": "D-Pad Left.svg", "dpad_right": "D-Pad Right.svg",
    "back": "View.svg", "start": "Menu.svg", "guide": "Home.svg", "share": "Share.svg",
    // Stick axes and directions by canonical input name, for the bindings table.
    "left_stick_y": "Left Stick Up-Down.svg", "left_stick_x": "Left Stick Left-Right.svg",
    "left_stick_up": "Left Stick Up.svg", "left_stick_down": "Left Stick Down.svg",
    "left_stick_left": "Left Stick Left.svg", "left_stick_right": "Left Stick Right.svg",
    "right_stick_y": "Right Stick Up-Down.svg", "right_stick_x": "Right Stick Left-Right.svg",
    "right_stick_up": "Right Stick Up.svg", "right_stick_down": "Right Stick Down.svg",
    "right_stick_left": "Right Stick Left.svg", "right_stick_right": "Right Stick Right.svg"
  })

  readonly property color tint: active ? activeColor
    : (reserved ? accentColor
    : (faceColors[controlKey] !== undefined ? faceColors[controlKey] : contentColor))

  implicitWidth: size
  implicitHeight: size
  visible: glyphFiles[controlKey] !== undefined

  Image {
    id: glyph
    anchors.fill: parent
    visible: false
    source: icon.glyphFiles[icon.controlKey]
              ? "svgs/buttons/" + icon.glyphFiles[icon.controlKey].replace(/ /g, "%20")
              : ""
    fillMode: Image.PreserveAspectFit
    sourceSize.width: Math.ceil(icon.size * 4)
    sourceSize.height: Math.ceil(icon.size * 4)
    smooth: true
    mipmap: true
  }

  ColorOverlay {
    anchors.fill: glyph
    source: glyph
    color: icon.tint
  }
}
