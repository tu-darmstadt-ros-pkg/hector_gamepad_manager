import QtQuick
import QtQuick.Layouts

// Schematic of an Xbox-style gamepad with callouts describing the function bound to each control.
// The controller artwork is the bundled xbox-series-controller.svg. Controls that sit together
// on the device share a callout group: the group is a plate stacking one glyph (GamepadButtonIcon)
// per control beside a text row per bound action, with a spine along the edge facing the artwork
// and a single right-angled connector from the spine's midpoint to the group's anchor on the
// drawing.
// This control is data-only: it takes plain JS objects and has no ROS or config dependency.
//
// Qt6 port of Hector.Controls.GamepadView from hector_qml_controls. It differs in two points: the
// Hector.Utils Units dependency is covered by the local pt() below, and activeControls lets the
// diagram show live gamepad state rather than only the static mapping. Keep the two in sync.
Item {
  id: control

  //! Map of control key -> array of rows {badge: string, text: string, key: string}.
  //! `key` is optional and names the keyboard key for that row specifically, which matters where
  //! one control carries several bindings reached by different keys - a stick bound as two axes
  //! shows "W / S" beside its drive row and "A / D" beside its steer row. Without it the row falls
  //! back to the control's entry in controlKeys.
  //! Keys: "a","b","x","y","lb","rb","lt","rt","back","start","guide","share","lstick",
  //! "lstick_click","rstick","rstick_click","dpad","dpad_up","dpad_down","dpad_left","dpad_right".
  //! Controls absent from the map are left out of their group; a group without any labelled control
  //! is not drawn at all.
  property var controlLabels: ({})

  //! Control keys to highlight as reserved config-switch buttons (their glyph is drawn in
  //! accentColor).
  property var reservedControls: []

  //! Control keys currently deflected or pressed, drawn in activeColor. Leave empty for a purely
  //! static mapping diagram.
  property var activeControls: []

  //! Map of control key -> the keyboard key that drives it, e.g. { "lt": "Q", "a": "↓" }. Drawn as
  //! an outlined keycap beside the control's glyph, and used as the fallback for rows that name no
  //! key of their own. Leave empty when a real gamepad is being described, where there is no
  //! keyboard key to name.
  property var controlKeys: ({})

  //! Color of the callout text, spines and connectors. Also tints the monochrome button glyphs.
  property color contentColor: "#37474F"

  //! Highlight color for reserved config-switch buttons and action modifier badges.
  property color accentColor: "#2196F3"

  //! Tint for controls listed in activeControls. Takes precedence over every other glyph color.
  property color activeColor: "#43A047"

  //! Plate drawn behind each group, and as a halo under the connectors, so they stay readable on a
  //! transparent or busy background. Set to "transparent" to draw the groups bare.
  property color labelBackgroundColor: Qt.rgba(1, 1, 1, 0.8)

  //! Point size in pixels, mirroring Hector.Utils' Units.pt() so the ported layout constants below
  //! keep their original proportions without pulling in that module.
  function pt(pointSize) { return pointSize * ptMetrics.height / 1000 }

  FontMetrics {
    id: ptMetrics
    font.pointSize: 1000
  }

  QtObject {
    id: d

    // SVG viewBox: "0 0 1728.0194 1202.2335".
    readonly property real vbX: 0
    readonly property real vbY: 0
    readonly property real vbW: 1728.0194
    readonly property real vbH: 1202.2335
    readonly property real aspect: vbW / vbH

    // Callout groups. `keys` stack top to bottom. `anchor` is the point on the artwork the group's
    // connector ends at, in viewBox coordinates. `side` is the group edge the spine runs along, and
    // always faces the artwork: a group beside the drawing gets a vertical spine, one above or
    // below it a horizontal one. The connector leaves the spine's midpoint away from the group,
    // turns to the anchor's height and runs in; `turnX` sets the viewBox x it turns at and defaults
    // to the anchor's own x, which collapses the last leg. It only needs setting to route a
    // connector around the controller body.
    readonly property var groups: [
      { band: "topLeft", side: "right", turnX: 280, keys: ["lt", "lb"],
        anchor: { x: 400, y: 42 } },
      { band: "topCenter", side: "bottom", keys: ["guide", "back", "start", "share"],
        anchor: { x: 860, y: 280.0 } },
      { band: "topRight", side: "left", turnX: 1450, keys: ["rt", "rb"],
        anchor: { x: 1320, y: 42 } },
      { band: "left", side: "right", keys: ["lstick", "lstick_click"],
        anchor: { x: 420, y: 340 } },
      { band: "left", side: "right", turnX: 250,
        keys: ["dpad_up", "dpad_down", "dpad_left", "dpad_right", "dpad"],
        anchor: { x: 630, y: 620 } },
      { band: "right", side: "left", keys: ["y", "x", "b", "a"],
        anchor: { x: 1310.0, y: 350 } },
      { band: "bottom", side: "top", keys: ["rstick", "rstick_click"],
        anchor: { x: 1090, y: 600 } }
    ]

    // Painted image geometry (PreserveAspectFit, centered) with margins on all sides for callouts.
    property real imgW: Math.min(control.width * 0.42, control.height * 0.5 * aspect)
    property real imgH: imgW / aspect
    property real imgX: (control.width - imgW) / 2
    property real imgY: (control.height - imgH) / 2

    property real margin: control.pt(6)
    property real leaderGap: control.pt(10)
    property real badgeSize: control.pt(26)
    property real maxTextWidth: control.pt(130)
    property real spineWidth: control.pt(2)
    property real spineGap: control.pt(6)
    property real groupPadding: control.pt(5)
    property real lineWidth: control.pt(1.5)
    property real haloWidth: control.pt(2)
    property real dotRadius: control.pt(2.5)
    property real plateRadius: control.pt(3)

    // Function-label font: larger and bold so the bindings read clearly next to the glyphs.
    readonly property font labelFont: Qt.font({ pointSize: 10, bold: true })

    // Modifier-badge font: small enough to sit beside a label without competing with it.
    readonly property font badgeFont: Qt.font({ pointSize: 8 })

    //! Keycap font: bold so a single letter stays legible inside its outline, but small enough
    //! that the cap does not compete with the control's own glyph.
    readonly property font keycapFont: Qt.font({ pointSize: 8, bold: true })

    // Vertical center of the band above the artwork.
    property real topBandY: imgY * 0.5

    function isReserved(key) { return control.reservedControls.indexOf(key) >= 0 }

    function isActive(key) { return control.activeControls.indexOf(key) >= 0 }

    // Map a viewBox coordinate to control-item pixels.
    function pxX(vx) { return imgX + (vx - vbX) / vbW * imgW }
    function pxY(vy) { return imgY + (vy - vbY) / vbH * imgH }

    // One entry per bound action, flattened over the group's keys in stacking order, each carrying
    // the grid row it occupies and the keycap that drives it. A row may name its own keycap - an
    // axis binding knows which keys move that particular axis, which the control as a whole does
    // not - and falls back to the control's cap otherwise.
    //
    // When the group has no bound action at all, its controls each get one blank row so the
    // diagram can still name their keys. Groups that do have bindings show only those, which is
    // what keeps an alias like the undirected "dpad" from adding an empty fifth row beneath the
    // four directions that are actually bound.
    function rowsFor(keys) {
      var out = []
      for (var i = 0; i < keys.length; ++i) {
        var rows = control.controlLabels[keys[i]]
        if (!rows)
          continue
        for (var j = 0; j < rows.length; ++j)
          out.push({ key: keys[i], badge: rows[j].badge || "", text: rows[j].text || "",
                     cap: rows[j].key || control.controlKeys[keys[i]] || "", row: out.length })
      }
      if (out.length > 0)
        return out
      for (var k = 0; k < keys.length; ++k) {
        var cap = control.controlKeys[keys[k]]
        if (cap)
          out.push({ key: keys[k], badge: "", text: "", cap: cap, row: out.length })
      }
      return out
    }

    // One entry per drawn glyph, carrying the grid rows it has to span. Derived from rowsFor so
    // the two models can never disagree about which row a control sits on.
    function badgesFor(keys) {
      return spanRuns(rowsFor(keys), function (row) { return row.key })
    }

    // One entry per keycap, spanning the consecutive rows that share it. A control whose actions
    // are all reached by the same key shows the cap once; a stick bound as two axes shows each
    // axis' own keys beside its own row.
    function capsFor(keys) {
      return spanRuns(rowsFor(keys), function (row) { return row.cap })
        .filter(function (run) { return run.value !== "" })
    }

    // Collapse consecutive rows with the same value into {value, key, row, span} runs.
    function spanRuns(rows, valueOf) {
      var out = []
      for (var i = 0; i < rows.length; ++i) {
        var value = valueOf(rows[i])
        if (out.length > 0 && out[out.length - 1].value === value &&
            out[out.length - 1].key === rows[i].key) {
          out[out.length - 1].span += 1
          continue
        }
        out.push({ value: value, key: rows[i].key, row: rows[i].row, span: 1 })
      }
      return out
    }

    function groupsInBand(band) {
      var out = []
      for (var i = 0; i < groups.length; ++i)
        if (groups[i].band === band && rowsFor(groups[i].keys).length > 0)
          out.push(groups[i])
      return out
    }

    // Keep a band fully on-screen, giving up its preferred position before it clips.
    function clampX(v, w) { return Math.max(margin, Math.min(v, control.width - w - margin)) }
    function clampY(v, h) { return Math.max(margin, Math.min(v, control.height - h - margin)) }
  }

  Image {
    id: gamepadImage
    source: "svgs/xbox-series-controller.svg"
    fillMode: Image.PreserveAspectFit
    smooth: true
    x: d.imgX
    y: d.imgY
    width: d.imgW
    height: d.imgH
    sourceSize.width: Math.ceil(d.imgW)
    sourceSize.height: Math.ceil(d.imgH)
  }

  ColumnLayout {
    id: topLeftBand
    spacing: control.pt(12)
    x: d.clampX(d.imgX - d.leaderGap - width, width)
    y: d.clampY(d.topBandY - height / 2, height)
    Repeater { id: topLeftRep; model: d.groupsInBand("topLeft"); delegate: groupComponent }
  }

  ColumnLayout {
    id: topCenterBand
    spacing: control.pt(12)
    x: d.clampX(d.imgX + d.imgW / 2 - width / 2, width)
    y: d.clampY(d.topBandY - height / 2, height)
    Repeater { id: topCenterRep; model: d.groupsInBand("topCenter"); delegate: groupComponent }
  }

  ColumnLayout {
    id: topRightBand
    spacing: control.pt(12)
    x: d.clampX(d.imgX + d.imgW + d.leaderGap, width)
    y: d.clampY(d.topBandY - height / 2, height)
    Repeater { id: topRightRep; model: d.groupsInBand("topRight"); delegate: groupComponent }
  }

  ColumnLayout {
    id: leftBand
    spacing: control.pt(12)
    x: d.clampX(d.imgX - d.leaderGap - width, width)
    y: d.clampY(control.height / 2 - height / 2, height)
    Repeater { id: leftRep; model: d.groupsInBand("left"); delegate: groupComponent }
  }

  ColumnLayout {
    id: rightBand
    spacing: control.pt(12)
    x: d.clampX(d.imgX + d.imgW + d.leaderGap, width)
    y: d.clampY(control.height / 2 - height / 2, height)
    Repeater { id: rightRep; model: d.groupsInBand("right"); delegate: groupComponent }
  }

  ColumnLayout {
    id: bottomBand
    spacing: control.pt(12)
    x: d.clampX(d.imgX + d.imgW / 2 - width / 2, width)
    y: d.clampY(d.imgY + 0.8 * d.imgH + d.leaderGap, height)
    Repeater { id: bottomRep; model: d.groupsInBand("bottom"); delegate: groupComponent }
  }

  // Connectors from every group's dot to its anchor. Drawn over the bands so a connector stays
  // joined to the dot it leaves rather than disappearing under its own group's plate.
  Canvas {
    id: connectorCanvas
    anchors.fill: parent

    // Repaint whenever any band's geometry settles.
    property real relayout: control.width + control.height
      + topLeftBand.x + topLeftBand.y + topLeftBand.width + topLeftBand.height
      + topCenterBand.x + topCenterBand.y + topCenterBand.width + topCenterBand.height
      + topRightBand.x + topRightBand.y + topRightBand.width + topRightBand.height
      + leftBand.x + leftBand.y + leftBand.width + leftBand.height
      + rightBand.x + rightBand.y + rightBand.width + rightBand.height
      + bottomBand.x + bottomBand.y + bottomBand.width + bottomBand.height
    onRelayoutChanged: requestPaint()

    Connections {
      target: control
      function onControlLabelsChanged() { connectorCanvas.requestPaint() }
      function onContentColorChanged() { connectorCanvas.requestPaint() }
      function onLabelBackgroundColorChanged() { connectorCanvas.requestPaint() }
    }

    readonly property var repeaters: [topLeftRep, topCenterRep, topRightRep,
                                      leftRep, rightRep, bottomRep]

    // The spine runs the length of the group's plate, just off the edge facing the artwork.
    function strokeSpine(ctx, group) {
      var from = group.verticalSpine ? group.mapToItem(connectorCanvas, group.dotX, 0)
                                     : group.mapToItem(connectorCanvas, 0, group.dotY)
      var to = group.verticalSpine ? group.mapToItem(connectorCanvas, group.dotX, group.height)
                                   : group.mapToItem(connectorCanvas, group.width, group.dotY)
      ctx.beginPath()
      ctx.moveTo(from.x, from.y)
      ctx.lineTo(to.x, to.y)
      ctx.stroke()
    }

    function fillDot(ctx, group, radius) {
      var dot = group.mapToItem(connectorCanvas, group.dotX, group.dotY)
      ctx.beginPath()
      ctx.arc(dot.x, dot.y, radius, 0, 2 * Math.PI)
      ctx.fill()
    }

    // Right-angled route from the dot to the anchor, turning at turnX. Off a vertical spine it
    // leaves sideways, turns to the anchor's height and runs in. Off a horizontal spine it leaves
    // squarely away from the group, crosses halfway between group and anchor, and drops in.
    // Degenerate segments collapse on their own, so a turnX left at the anchor's own x costs the
    // last leg and yields a plain elbow.
    function strokeConnector(ctx, group) {
      var dot = group.mapToItem(connectorCanvas, group.dotX, group.dotY)
      var def = group.groupDef
      var anchorX = d.pxX(def.anchor.x)
      var anchorY = d.pxY(def.anchor.y)
      var turnX = def.turnX !== undefined ? d.pxX(def.turnX) : anchorX
      ctx.beginPath()
      ctx.moveTo(dot.x, dot.y)
      if (group.verticalSpine) {
        ctx.lineTo(turnX, dot.y)
      } else {
        var turnY = (dot.y + anchorY) / 2
        ctx.lineTo(dot.x, turnY)
        ctx.lineTo(turnX, turnY)
      }
      ctx.lineTo(turnX, anchorY)
      ctx.lineTo(anchorX, anchorY)
      ctx.stroke()
    }

    // `grow` widens every stroke so the same pass can lay down a halo under the real lines. Spine,
    // dot and connector all go through it, so the whole run reads as one line on any background.
    function pass(ctx, color, grow) {
      ctx.strokeStyle = color
      ctx.fillStyle = color
      ctx.lineJoin = "round"
      ctx.lineCap = "round"
      for (var r = 0; r < repeaters.length; ++r) {
        for (var i = 0; i < repeaters[r].count; ++i) {
          var group = repeaters[r].itemAt(i)
          if (!group)
            continue
          ctx.lineWidth = d.spineWidth + grow
          strokeSpine(ctx, group)
          ctx.lineWidth = d.lineWidth + grow
          strokeConnector(ctx, group)
          fillDot(ctx, group, d.dotRadius + grow / 2)
        }
      }
    }

    onPaint: {
      var ctx = getContext("2d")
      ctx.reset()
      pass(ctx, control.labelBackgroundColor, 2 * d.haloWidth)
      pass(ctx, control.contentColor, 0)
    }
  }

  // One callout group: a plate holding a grid of glyphs and action rows, with a spine along the
  // edge facing the artwork and the connector's dot at that spine's midpoint.
  Component {
    id: groupComponent
    Item {
      id: group

      property var groupDef: modelData
      readonly property string side: groupDef.side
      // A vertical spine sits left or right of the rows, a horizontal one above or below them.
      readonly property bool verticalSpine: side === "left" || side === "right"
      // Glyphs hug the spine where it runs alongside the rows, and otherwise lead them.
      readonly property bool glyphsRight: side === "right"

      readonly property real spineSpace: d.spineGap + d.spineWidth

      implicitWidth: grid.implicitWidth + 2 * d.groupPadding + (verticalSpine ? spineSpace : 0)
      implicitHeight: grid.implicitHeight + 2 * d.groupPadding + (verticalSpine ? 0 : spineSpace)

      Layout.preferredWidth: implicitWidth
      Layout.preferredHeight: implicitHeight
      Layout.alignment: verticalSpine ? (glyphsRight ? Qt.AlignRight : Qt.AlignLeft)
                                      : Qt.AlignHCenter

      // Spine center, and with it the connector's origin, in group coordinates. The spine sits in
      // the strip outside the plate so that it, the dot and the connector all share the canvas'
      // halo - on the plate it would be the only part of the run without one.
      readonly property real dotX: !verticalSpine ? width / 2
                                 : (side === "right" ? width - d.spineWidth / 2 : d.spineWidth / 2)
      readonly property real dotY: verticalSpine ? height / 2
                                 : (side === "bottom" ? height - d.spineWidth / 2 : d.spineWidth / 2)

      Rectangle {
        radius: d.plateRadius
        color: control.labelBackgroundColor
        x: group.side === "left" ? group.spineSpace : 0
        y: group.side === "top" ? group.spineSpace : 0
        width: group.width - (group.verticalSpine ? group.spineSpace : 0)
        height: group.height - (group.verticalSpine ? 0 : group.spineSpace)
      }

      GridLayout {
        id: grid
        // Glyph, keycap and action rows. The keycap column collapses to nothing when the caller
        // supplies no controlKeys, leaving the original two-column layout.
        columns: 3
        rowSpacing: control.pt(4)
        columnSpacing: control.pt(6)
        x: group.side === "left" ? d.groupPadding + group.spineSpace : d.groupPadding
        y: group.side === "top" ? d.groupPadding + group.spineSpace : d.groupPadding
        width: group.width - 2 * d.groupPadding - (group.verticalSpine ? group.spineSpace : 0)
        height: group.height - 2 * d.groupPadding - (group.verticalSpine ? 0 : group.spineSpace)

        // Glyph column. A glyph spans all of its control's action rows so it ends up vertically
        // centered on them.
        Repeater {
          model: d.badgesFor(group.groupDef.keys)
          delegate: GamepadButtonIcon {
            Layout.row: modelData.row
            Layout.rowSpan: modelData.span
            Layout.column: group.glyphsRight ? 2 : 0
            Layout.alignment: Qt.AlignVCenter | (group.glyphsRight ? Qt.AlignRight : Qt.AlignLeft)
            controlKey: modelData.value
            contentColor: control.contentColor
            accentColor: control.accentColor
            reserved: d.isReserved(modelData.key)
            active: d.isActive(modelData.key)
            activeColor: control.activeColor
            size: d.badgeSize
          }
        }

        // Keycap column: which keyboard key reaches the control. Drawn as an outlined cap so it
        // reads as a key rather than as another part of the gamepad. Sits between the glyph and
        // the action rows on both sides, so it stays next to the control it belongs to.
        Repeater {
          model: d.capsFor(group.groupDef.keys)
          delegate: Rectangle {
            readonly property string keyLabel: modelData.value

            Layout.row: modelData.row
            Layout.rowSpan: modelData.span
            Layout.column: 1
            Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
            implicitWidth: keyText.implicitWidth + 2 * control.pt(3)
            implicitHeight: keyText.implicitHeight + control.pt(2)
            radius: control.pt(2)
            color: "transparent"
            border.width: Math.max(1, control.pt(0.75))
            border.color: d.isActive(modelData.key) ? control.activeColor : control.contentColor

            Text {
              id: keyText
              anchors.centerIn: parent
              text: parent.keyLabel
              color: parent.border.color
              font: d.keycapFont
            }
          }
        }

        // Action column: one row per bound action, flowing away from the spine.
        Repeater {
          model: d.rowsFor(group.groupDef.keys)
          delegate: Item {
            Layout.row: modelData.row
            Layout.column: group.glyphsRight ? 0 : 2
            Layout.alignment: Qt.AlignVCenter | (group.glyphsRight ? Qt.AlignRight : Qt.AlignLeft)
            implicitWidth: rowContent.implicitWidth
            implicitHeight: rowContent.implicitHeight

            // Natural (unwrapped) text width, measured without feeding back into the layout.
            TextMetrics { id: metrics; font: d.labelFont; text: modelData.text }

            Row {
              id: rowContent
              spacing: control.pt(3)
              // Rows left of the glyphs flow right-to-left so the modifier badge keeps hugging the
              // glyph even when the description wraps to several lines.
              layoutDirection: group.glyphsRight ? Qt.RightToLeft : Qt.LeftToRight

              Text {
                visible: modelData.badge.length > 0
                text: modelData.badge
                color: control.accentColor
                font: d.badgeFont
                anchors.verticalCenter: parent.verticalCenter
              }

              Text {
                text: modelData.text
                color: control.contentColor
                font: d.labelFont
                width: Math.min(metrics.advanceWidth + control.pt(1), d.maxTextWidth)
                wrapMode: Text.WordWrap
                horizontalAlignment: group.glyphsRight ? Text.AlignRight : Text.AlignLeft
              }
            }
          }
        }
      }
    }
  }
}
