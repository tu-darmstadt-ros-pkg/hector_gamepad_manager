import QtQuick
import Qt5Compat.GraphicalEffects

// Button-glyph badge for a single gamepad control key. Renders the matching Zacksly controller icon
// (svgs/buttons, CC BY 3.0 - see svgs/ATTRIBUTION.md) tinted to fit the theme: face buttons take
// muted, colour-coded hues; every other control follows contentColor. Reserved config-switch
// buttons are drawn in accentColor.
//
// The glyphs are shape-positive with their label cut out as a transparent hole, so the label reads
// as whatever sits behind the badge.
Item {
  id: icon

  //! Control key, one of the keys in glyphFiles. Unknown keys render nothing.
  property string controlKey

  //! Tint for the monochrome (non-face) glyphs.
  property color contentColor: "#37474F"

  //! Tint applied to the whole badge when reserved.
  property color accentColor: "#2196F3"

  //! Draw the badge in accentColor to mark a reserved config-switch button.
  property bool reserved: false

  //! Tint applied to the whole badge while the control is deflected or pressed.
  property color activeColor: "#43A047"

  //! Draw the badge in activeColor to mark a control the operator is currently using.
  property bool active: false

  //! Badge edge length (the source glyphs are square).
  property real size: 20

  readonly property var faceColors: ({ "a": "#009E73", "b": "#C0392B", "x": "#0072B2", "y": "#E69F00" })
  readonly property var glyphFiles: ({
    "a": "A.svg", "b": "B.svg", "x": "X.svg", "y": "Y.svg",
    "lb": "Left Bumper.svg", "rb": "Right Bumper.svg",
    "lt": "Left Trigger.svg", "rt": "Right Trigger.svg",
    "lstick": "Left Stick.svg", "rstick": "Right Stick.svg",
    "lstick_click": "Left Stick Click.svg", "rstick_click": "Right Stick Click.svg",
    "dpad": "D-Pad.svg", "dpad_up": "D-Pad Up.svg", "dpad_down": "D-Pad Down.svg",
    "dpad_left": "D-Pad Left.svg", "dpad_right": "D-Pad Right.svg",
    "back": "View.svg", "start": "Menu.svg", "guide": "Home.svg", "share": "Share.svg"
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
