import QtQuick
import QtTest
import "../qml"

// Smoke tests: the imports resolve, the bundled SVGs load and the callout groups lay out.
TestCase {
  id: testCase
  name: "GamepadView"
  when: windowShown
  width: 900
  height: 600

  // Both are rebuilt in init(), so no state carries over between tests.
  readonly property var view: viewLoader.item
  readonly property var icon: iconLoader.item

  ItemFinder {
    id: finder
  }

  Loader {
    id: viewLoader
    sourceComponent: Component {
      GamepadView {
        width: testCase.width
        height: testCase.height
        controlLabels: ({
          "a": [{ badge: "", text: "Drive fast" }],
          "lstick": [{ badge: "", text: "Drive" }, { badge: "hold", text: "Boost" }],
          "back": [{ badge: "", text: "Switch mode" }],
          "share": [{ badge: "", text: "Invert steering" }],
          "dpad_up": [{ badge: "2x", text: "Flipper up" }]
        })
        reservedControls: ["back"]
      }
    }
  }

  Loader {
    id: iconLoader
    sourceComponent: Component {
      GamepadButtonIcon {
        controlKey: "a"
        size: 32
      }
    }
  }

  function init() {
    viewLoader.active = false
    viewLoader.active = true
    iconLoader.active = false
    iconLoader.active = true
  }

  function test_controller_artwork_loads() {
    var images = collectImages(view)
    verify(images.length > 0, "no Image found in GamepadView")
    for (var i = 0; i < images.length; ++i)
      compare(images[i].status, Image.Ready,
              "failed to load " + images[i].source + " - is the Qt SVG image plugin installed?")
  }

  function test_button_glyph_loads() {
    var images = collectImages(icon)
    verify(images.length > 0)
    compare(images[0].status, Image.Ready, "failed to load " + images[0].source)
  }

  // A renamed or missing SVG would render an empty badge.
  function test_every_glyph_file_resolves() {
    for (var key in icon.glyphFiles) {
      icon.controlKey = key
      var images = collectImages(icon)
      compare(images[0].status, Image.Ready,
              "glyph for '" + key + "' (" + icon.glyphFiles[key] + ") did not load")
    }
  }

  function test_share_has_a_glyph() {
    verify(icon.glyphFiles["share"] !== undefined,
           "the Share button needs a glyph or its bindings render without a badge")
  }

  function test_keycaps_are_drawn_next_to_their_glyph() {
    view.controlKeys = ({ "a": "A", "lt": "W" })
    verify(keycapLabels(view).indexOf("A") >= 0, "no keycap drawn for the A button")
  }

  function test_keycaps_show_without_any_bindings() {
    view.controlLabels = ({})
    view.controlKeys = ({ "a": "A", "lt": "W" })
    var caps = keycapLabels(view)
    verify(caps.indexOf("A") >= 0, "keycaps vanish when nothing is bound")
    verify(caps.indexOf("W") >= 0)
  }

  function test_rows_carry_their_own_keycap() {
    view.controlKeys = ({ "lstick": "fallback" })
    view.controlLabels = ({
      "lstick": [{ badge: "", text: "Drive forward/backward", key: "W / S" },
                 { badge: "", text: "Steer", key: "A / D" }]
    })
    var caps = keycapLabels(view)
    verify(caps.indexOf("W / S") >= 0, "the drive row must show its own axis keys")
    verify(caps.indexOf("A / D") >= 0, "the steer row must show its own axis keys")
    verify(caps.indexOf("fallback") < 0, "a row naming its key must not fall back to the control")
  }

  function test_rows_without_a_key_fall_back_to_the_control() {
    view.controlKeys = ({ "a": "↓" })
    view.controlLabels = ({ "a": [{ badge: "", text: "Drive slowly" }] })
    compare(keycapLabels(view), ["↓"])
  }

  // "dpad" is an alias for the four directions and must not add an empty row once they are bound.
  function test_an_alias_adds_no_empty_row_beside_bound_siblings() {
    view.controlKeys = ({ "dpad": "T/G F/H", "dpad_up": "T", "dpad_down": "G" })
    view.controlLabels = ({
      "dpad_up": [{ badge: "", text: "Flippers up", key: "T" }],
      "dpad_down": [{ badge: "", text: "Flippers down", key: "G" }]
    })
    var caps = keycapLabels(view)
    verify(caps.indexOf("T/G F/H") < 0, "the undirected d-pad added an orphan row")
    compare(caps.length, 2)
  }

  function test_no_keycaps_when_none_are_supplied() {
    view.controlKeys = ({})
    view.controlLabels = ({})
    compare(collectIcons(view).length, 0,
            "with neither bindings nor keycaps the diagram draws no callouts at all")
    compare(keycapLabels(view).length, 0)
  }

  function test_groups_are_laid_out() {
    verify(view.width > 0 && view.height > 0)
    var icons = collectIcons(view)
    verify(icons.length > 0, "no button glyphs were drawn for the given controlLabels")
  }

  function test_fits_needs_room_for_the_callouts() {
    view.width = 300
    view.height = 200
    tryVerify(function () { return !view.fits }, 1000, "the callouts cannot fit into 300x200")
    view.width = 1600
    view.height = 900
    tryVerify(function () { return view.fits }, 1000, "a few callouts fit into 1600x900")
  }

  function test_active_controls_retint_the_glyph() {
    var before = icon.tint
    icon.active = true
    icon.activeColor = "#123456"
    compare(icon.tint, "#123456")
    icon.active = false
    compare(icon.tint, before)
  }

  // Item.visible is always false inside a TestCase, so match the keycap by its keyLabel property.
  function keycapLabels(item) {
    return finder.collect(item, function (child) { return child.keyLabel !== undefined })
      .map(function (cap) { return cap.keyLabel })
      .filter(function (label) { return label !== "" })
  }

  function collectImages(item) {
    return finder.collect(item, function (child) { return child instanceof Image })
  }

  // GamepadButtonIcon is a QML type, so match on a property only it declares.
  function collectIcons(item) {
    return finder.collect(item, function (child) { return child.controlKey !== undefined })
  }
}
