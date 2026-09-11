import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import RQml.Elements

// Controller drawing with a callout per control group listing the actions bound to it.
// Qt5 counterpart: Hector.Controls.GamepadView in hector_qml_controls.
Item {
  id: control

  //! Control key -> [{ badge, text, key }]. `key` is the row's keycap; rows without one use
  //! controlKeys. Keys: a b x y lb rb lt rt back start guide share lstick lstick_click rstick
  //! rstick_click dpad dpad_up dpad_down dpad_left dpad_right.
  property var controlLabels: ({})

  //! Control keys of the config-switch buttons, drawn in accentColor.
  property var reservedControls: []

  //! Control keys currently pressed or deflected, drawn in activeColor.
  property var activeControls: []

  //! Control key -> keyboard key, e.g. { "lt": "Q" }, drawn as a keycap beside the glyph.
  property var controlKeys: ({})

  //! Callout text, lines and monochrome glyphs.
  property color contentColor: palette.text

  //! Reserved buttons and modifier badges.
  property color accentColor: palette.highlight

  //! Glyphs in activeControls. Overrides every other glyph color.
  property color activeColor: palette.highlight

  //! Plate behind each group and halo under the connectors.
  property color labelBackgroundColor: palette.base

  //! The callouts fit at their preferred positions, clear of each other and of the artwork.
  readonly property bool fits: d.calloutsFit()

  //! Layout unit: a tenth of the font's line height, so the geometry scales with the font.
  function unit(size) { return size * fontMetrics.height / 10 }

  FontMetrics {
    id: fontMetrics
  }

  QtObject {
    id: d

    // SVG viewBox: "0 0 1728.0194 1202.2335".
    readonly property real vbX: 0
    readonly property real vbY: 0
    readonly property real vbW: 1728.0194
    readonly property real vbH: 1202.2335
    readonly property real aspect: vbW / vbH

    // Callout groups. `keys` stack top to bottom, `anchor` is where the connector ends on the
    // artwork (viewBox coordinates), `side` is the group edge facing the artwork, and `turnX` is
    // the viewBox x where the connector turns (default: the anchor's x).
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

    // Artwork geometry. The artwork takes the width the side callouts leave, capped by the height,
    // and moves down when the top callouts need more room than centering leaves them.
    property real imgW: Math.max(control.unit(150), Math.min(
      control.width - sideWidth - 2 * leaderGap - 2 * margin, control.height * 0.5 * aspect))
    // The artwork is centered, so both sides need the room of the widest column.
    readonly property real sideWidth: 2 * Math.max(topLeftBand.width, leftBand.width,
                                                   topRightBand.width, rightBand.width)
    property real imgH: imgW / aspect
    property real imgX: (control.width - imgW) / 2
    property real imgY: Math.max((control.height - imgH) / 2, margin + topBandsHeight + leaderGap)
    readonly property real topBandsHeight: Math.max(topLeftBand.height, topCenterBand.height,
                                                    topRightBand.height)

    property real margin: control.unit(6)
    property real leaderGap: control.unit(10)
    property real badgeSize: control.unit(16)
    property real maxTextWidth: control.unit(130)
    property real spineWidth: control.unit(2)
    property real spineGap: control.unit(6)
    property real groupPadding: control.unit(5)
    property real lineWidth: control.unit(1.5)
    property real haloWidth: control.unit(2)
    property real dotRadius: control.unit(2.5)
    property real plateRadius: control.unit(3)

    // Vertical center of the band above the artwork.
    property real topBandY: imgY * 0.5

    function isReserved(key) { return control.reservedControls.indexOf(key) >= 0 }

    function isActive(key) { return control.activeControls.indexOf(key) >= 0 }

    // Map a viewBox coordinate to control-item pixels.
    function pxX(vx) { return imgX + (vx - vbX) / vbW * imgW }
    function pxY(vy) { return imgY + (vy - vbY) / vbH * imgH }

    // One row per bound action with its grid row and keycap. A group without bindings gets one
    // blank row per control that has a keycap.
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

    // One glyph per control, spanning that control's rows.
    function badgesFor(keys) {
      return spanRuns(rowsFor(keys), function (row) { return row.key })
    }

    // One keycap per run of consecutive rows that share it.
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

    // Keep a band on-screen.
    function clampX(v, w) { return Math.max(margin, Math.min(v, control.width - w - margin)) }
    function clampY(v, h) { return Math.max(margin, Math.min(v, control.height - h - margin)) }

    // Every band at its preferred position stays inside the view and clear of the other bands and
    // the artwork. The bottom band sits between the grips, over the artwork, by design.
    function calloutsFit() {
      var bands = [topLeftBand, topCenterBand, topRightBand, leftBand, rightBand, bottomBand]
      var artwork = Qt.rect(imgX, imgY, imgW, imgH)
      var placed = []
      for (var i = 0; i < bands.length; ++i) {
        var band = bands[i]
        if (band.width <= 0 || band.height <= 0)
          continue
        var rect = Qt.rect(band.preferredX, band.preferredY, band.width, band.height)
        // Half a pixel of slack: the widest side column sits exactly on the margin.
        if (rect.x < margin - 0.5 || rect.y < margin - 0.5
            || rect.x + rect.width > control.width - margin + 0.5
            || rect.y + rect.height > control.height - margin + 0.5)
          return false
        if (band !== bottomBand && overlaps(rect, artwork))
          return false
        for (var j = 0; j < placed.length; ++j)
          if (overlaps(rect, placed[j]))
            return false
        placed.push(rect)
      }
      return true
    }

    function overlaps(a, b) {
      return a.x < b.x + b.width && b.x < a.x + a.width
          && a.y < b.y + b.height && b.y < a.y + a.height
    }
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
    readonly property real preferredX: d.imgX - d.leaderGap - width
    readonly property real preferredY: d.topBandY - height / 2
    spacing: control.unit(12)
    x: d.clampX(preferredX, width)
    y: d.clampY(preferredY, height)
    Repeater { id: topLeftRep; model: d.groupsInBand("topLeft"); delegate: groupComponent }
  }

  ColumnLayout {
    id: topCenterBand
    readonly property real preferredX: d.imgX + d.imgW / 2 - width / 2
    readonly property real preferredY: d.topBandY - height / 2
    spacing: control.unit(12)
    x: d.clampX(preferredX, width)
    y: d.clampY(preferredY, height)
    Repeater { id: topCenterRep; model: d.groupsInBand("topCenter"); delegate: groupComponent }
  }

  ColumnLayout {
    id: topRightBand
    readonly property real preferredX: d.imgX + d.imgW + d.leaderGap
    readonly property real preferredY: d.topBandY - height / 2
    spacing: control.unit(12)
    x: d.clampX(preferredX, width)
    y: d.clampY(preferredY, height)
    Repeater { id: topRightRep; model: d.groupsInBand("topRight"); delegate: groupComponent }
  }

  ColumnLayout {
    id: leftBand
    readonly property real preferredX: d.imgX - d.leaderGap - width
    readonly property real preferredY: control.height / 2 - height / 2
    spacing: control.unit(12)
    x: d.clampX(preferredX, width)
    y: d.clampY(preferredY, height)
    Repeater { id: leftRep; model: d.groupsInBand("left"); delegate: groupComponent }
  }

  ColumnLayout {
    id: rightBand
    readonly property real preferredX: d.imgX + d.imgW + d.leaderGap
    readonly property real preferredY: control.height / 2 - height / 2
    spacing: control.unit(12)
    x: d.clampX(preferredX, width)
    y: d.clampY(preferredY, height)
    Repeater { id: rightRep; model: d.groupsInBand("right"); delegate: groupComponent }
  }

  ColumnLayout {
    id: bottomBand
    readonly property real preferredX: d.imgX + d.imgW / 2 - width / 2
    readonly property real preferredY: d.imgY + 0.8 * d.imgH + d.leaderGap
    spacing: control.unit(12)
    x: d.clampX(preferredX, width)
    y: d.clampY(preferredY, height)
    Repeater { id: bottomRep; model: d.groupsInBand("bottom"); delegate: groupComponent }
  }

  // Connectors from each group to its anchor. Drawn above the bands, or a group's plate would hide
  // the start of its connector.
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

    // Line along the group's edge facing the artwork.
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

    // Right-angled route from the dot to the anchor, turning at turnX. Zero-length segments are
    // harmless, so an unset turnX gives a plain elbow.
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

    // Strokes every group. A positive `grow` widens the lines, which is how the halo is drawn.
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

  // One callout group: glyph, keycap and action columns, with a spine facing the artwork.
  Component {
    id: groupComponent
    Item {
      id: group

      property var groupDef: modelData
      readonly property string side: groupDef.side
      // A vertical spine sits left or right of the rows, a horizontal one above or below them.
      readonly property bool verticalSpine: side === "left" || side === "right"
      // Glyphs sit next to a spine on the right, otherwise at the start of the row.
      readonly property bool glyphsRight: side === "right"

      readonly property real spineSpace: d.spineGap + d.spineWidth

      implicitWidth: grid.implicitWidth + 2 * d.groupPadding + (verticalSpine ? spineSpace : 0)
      implicitHeight: grid.implicitHeight + 2 * d.groupPadding + (verticalSpine ? 0 : spineSpace)

      Layout.preferredWidth: implicitWidth
      Layout.preferredHeight: implicitHeight
      Layout.alignment: verticalSpine ? (glyphsRight ? Qt.AlignRight : Qt.AlignLeft)
                                      : Qt.AlignHCenter

      // Connector origin: the spine's center, just outside the plate.
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
        // Glyph, keycap and action columns. The keycap column is empty without controlKeys.
        columns: 3
        rowSpacing: control.unit(4)
        columnSpacing: control.unit(6)
        x: group.side === "left" ? d.groupPadding + group.spineSpace : d.groupPadding
        y: group.side === "top" ? d.groupPadding + group.spineSpace : d.groupPadding
        width: group.width - 2 * d.groupPadding - (group.verticalSpine ? group.spineSpace : 0)
        height: group.height - 2 * d.groupPadding - (group.verticalSpine ? 0 : group.spineSpace)

        // Glyph column. A glyph spans its control's rows, which centers it on them.
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

        // Keycap column: the keyboard key that drives the control.
        Repeater {
          model: d.capsFor(group.groupDef.keys)
          delegate: Rectangle {
            readonly property string keyLabel: modelData.value

            Layout.row: modelData.row
            Layout.rowSpan: modelData.span
            Layout.column: 1
            Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
            implicitWidth: keyText.implicitWidth + 2 * control.unit(3)
            implicitHeight: keyText.implicitHeight + control.unit(2)
            radius: control.unit(2)
            color: "transparent"
            border.width: Math.max(1, control.unit(0.75))
            border.color: d.isActive(modelData.key) ? control.activeColor : control.contentColor

            Caption {
              id: keyText
              anchors.centerIn: parent
              text: parent.keyLabel
              color: parent.border.color
              font.bold: true
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
            TextMetrics { id: metrics; font: actionText.font; text: modelData.text }

            Row {
              id: rowContent
              spacing: control.unit(3)
              // Right-to-left when left of the glyphs, so the badge stays next to the glyph when
              // the text wraps.
              layoutDirection: group.glyphsRight ? Qt.RightToLeft : Qt.LeftToRight

              Caption {
                visible: modelData.badge.length > 0
                text: modelData.badge
                color: control.accentColor
                anchors.verticalCenter: parent.verticalCenter
              }

              Label {
                id: actionText
                text: modelData.text
                color: control.contentColor
                font.bold: true
                width: Math.min(metrics.advanceWidth + control.unit(1), d.maxTextWidth)
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
