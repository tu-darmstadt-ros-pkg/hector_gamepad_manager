import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtQuick.Window
import Ros2
import RQml.Elements
import RQml.Fonts

// Drives a hector_gamepad_manager from the keyboard by publishing sensor_msgs/Joy, and shows what
// the active config binds each control to.
Rectangle {
  id: root

  property var kddockwidgets_min_size: Qt.size(420, 460)

  color: palette.base

  QtObject {
    id: d

    //! Namespace the selected joy_mapping topic lives in, e.g. "/robot1" (empty for a global one).
    readonly property string robotNamespace: {
      var topic = context.mappingTopic || ""
      var suffix = "/joy_mapping"
      return topic.length > suffix.length && topic.slice(-suffix.length) === suffix
             ? topic.slice(0, -suffix.length) : ""
    }

    readonly property string profileTopic:
      context.mappingTopic ? robotNamespace + "/joy_teleop_profile" : ""

    readonly property var publisher: {
      if (!context.joyTopic || !Ros2.isValidTopic(context.joyTopic))
        return null
      return Ros2.createPublisher(context.joyTopic, "sensor_msgs/msg/Joy", 1)
    }

    //! Latest mapping message as plain JS, null until one arrives.
    property var mapping: null

    //! Config the manager currently has active, as reported on joy_teleop_profile.
    property string activeProfile: ""

    // Switches made on a real gamepad or another panel only show up here.
    onActiveProfileChanged: gamepad.resetAxes()

    //! Config whose bindings are shown: the active one, or the mapping's default until it is known.
    readonly property string shownProfile: activeProfile || (mapping ? mapping.default_config : "")

    //! Config a switch has been requested for and not yet confirmed, "" when idle.
    readonly property string pendingProfile: switcher.pendingProfile

    //! Publish ticks left on a synthesized switch press, held for a few in case one is missed.
    property int switchTicksLeft: 0
    property string switchButton: ""

    // Message array fields arrive as a wrapper with toArray(); plain arrays pass straight through.
    function toArray(value) {
      if (!value)
        return []
      return value.toArray ? value.toArray() : value
    }

    function parseMapping(message) {
      return ({
        default_config: message.default_config,
        configs: toArray(message.configs).map(function (config) {
          return ({
            name: config.name,
            description: config.description,
            buttons: toArray(config.buttons).map(function (button) {
              return ({
                name: button.name,
                actions: toArray(button.actions).map(function (action) {
                  return ({
                    event: action.event,
                    function: action.function,
                    description: action.description
                  })
                })
              })
            }),
            axes: toArray(config.axes).map(function (axis) {
              return ({
                name: axis.name,
                function: axis.function,
                description: axis.description
              })
            })
          })
        }),
        config_switches: toArray(message.config_switches).map(function (config_switch) {
          return ({
            name: config_switch.name,
            config: config_switch.config,
            description: config_switch.description
          })
        })
      })
    }

    readonly property var configNames: mapping ? mapping.configs.map(function (config) {
      return config.name
    }) : []

    function findConfig(name) {
      if (!mapping)
        return null
      for (var i = 0; i < mapping.configs.length; ++i)
        if (mapping.configs[i].name === name)
          return mapping.configs[i]
      return null
    }

    // GamepadAction.event -> badge. EVENT_PRESS carries none.
    readonly property var eventBadge: ({ 0: "", 1: "2x", 2: "hold", 3: "release" })

    // Table sections in pad order, each listing its inputs in pad order.
    readonly property var tableSections: [
      { title: qsTr("Sticks"), names: [
          "left_stick_y", "left_stick_x", "left_stick_up", "left_stick_down", "left_stick_left",
          "left_stick_right", "left_stick_click",
          "right_stick_y", "right_stick_x", "right_stick_up", "right_stick_down",
          "right_stick_left", "right_stick_right", "right_stick_click"] },
      { title: qsTr("Triggers and bumpers"),
        names: ["left_trigger", "left_bumper", "right_trigger", "right_bumper"] },
      { title: qsTr("Face buttons"), names: ["y", "x", "b", "a"] },
      { title: qsTr("D-pad"), names: ["dpad_up", "dpad_down", "dpad_left", "dpad_right"] },
      { title: qsTr("System"), names: ["guide", "back", "start", "share"] }
    ]

    //! Canonical input name -> { section, order } in the table.
    readonly property var inputPositions: {
      var result = ({})
      tableSections.forEach(function (section, s) {
        section.names.forEach(function (name, i) {
          result[name] = { section: section.title, order: s * 100 + i }
        })
      })
      return result
    }

    //! One row per bound action of the shown config, in pad order: key, control, description.
    readonly property var bindings: {
      var rows = []
      var config = findConfig(shownProfile)
      if (config) {
        config.buttons.forEach(function (button) {
          button.actions.forEach(function (action) {
            rows.push(makeRow(button.name, eventBadge[action.event] || "",
                              action.description || action.function))
          })
        })
        config.axes.forEach(function (axis) {
          rows.push(makeRow(axis.name, "", axis.description || axis.function))
        })
      }
      if (mapping)
        mapping.config_switches.forEach(function (config_switch) {
          rows.push(makeRow(config_switch.name, "switch", "Switch to " + config_switch.config))
        })
      // Array.sort is not stable here, so the push order breaks ties: an input's actions keep the
      // config's order.
      rows.forEach(function (row, i) { row.sequence = i })
      rows.sort(function (a, b) { return a.order - b.order || a.sequence - b.sequence })
      return rows
    }

    //! Table rows: the bindings, with each double press folded into its control's press row.
    readonly property var tableRows: {
      var rows = []
      var pressRows = ({})
      bindings.forEach(function (binding) {
        var pressRow = pressRows[binding.name]
        if (binding.badge === "2x" && pressRow && pressRow.doublePress === "") {
          pressRow.doublePress = binding.text
          return
        }
        var row = Object.assign({ doublePress: "" }, binding)
        if (binding.badge === "")
          pressRows[binding.name] = row
        rows.push(row)
      })
      return rows
    }

    readonly property bool hasDoublePress: tableRows.some(function (row) {
      return row.doublePress !== ""
    })

    // Canonical input name -> GamepadView control key. Axes and their virtual buttons share one.
    readonly property var nameToControl: ({
      "a": "a", "b": "b", "x": "x", "y": "y",
      "left_bumper": "lb", "right_bumper": "rb",
      "back": "back", "start": "start", "guide": "guide", "share": "share",
      "left_stick_click": "lstick_click", "right_stick_click": "rstick_click",
      "left_stick_left": "lstick", "left_stick_right": "lstick",
      "left_stick_up": "lstick", "left_stick_down": "lstick",
      "right_stick_left": "rstick", "right_stick_right": "rstick",
      "right_stick_up": "rstick", "right_stick_down": "rstick",
      "left_trigger": "lt", "right_trigger": "rt",
      "dpad_left": "dpad_left", "dpad_right": "dpad_right",
      "dpad_up": "dpad_up", "dpad_down": "dpad_down",
      "left_stick_x": "lstick", "left_stick_y": "lstick",
      "right_stick_x": "rstick", "right_stick_y": "rstick"
    })

    //! GamepadView labels from the table rows. Rows without a place on the drawing are left out.
    readonly property var controlLabels: {
      var result = ({})
      bindings.forEach(function (row) {
        var key = nameToControl[row.name]
        if (!key)
          return
        if (!result[key])
          result[key] = []
        var rows = result[key]
        for (var i = 0; i < rows.length; ++i)
          if (rows[i].badge === row.badge && rows[i].text === row.text)
            return // an axis bound as axis and as virtual buttons would appear twice
        rows.push({ badge: row.badge, text: row.text, key: row.key })
      })
      return result
    }

    readonly property var reservedControls: {
      var keys = []
      if (mapping)
        mapping.config_switches.forEach(function (config_switch) {
          var key = nameToControl[config_switch.name]
          if (key && keys.indexOf(key) < 0)
            keys.push(key)
        })
      return keys
    }

    // Diagram control -> inputs whose keys label it when no binding row names a key.
    readonly property var controlKeySources: ({
      "a": ["a"], "b": ["b"], "x": ["x"], "y": ["y"],
      "lb": ["left_bumper"], "rb": ["right_bumper"],
      "lt": ["left_trigger"], "rt": ["right_trigger"],
      "back": ["back"], "start": ["start"], "guide": ["guide"], "share": ["share"],
      "lstick": ["left_stick_y", "left_stick_x"],
      "rstick": ["right_stick_y", "right_stick_x"],
      "lstick_click": ["left_stick_click"], "rstick_click": ["right_stick_click"],
      "dpad_up": ["dpad_up"], "dpad_down": ["dpad_down"],
      "dpad_left": ["dpad_left"], "dpad_right": ["dpad_right"]
    })

    //! Diagram control key -> keycap text.
    readonly property var controlKeys: {
      var result = ({})
      for (var key in controlKeySources) {
        var labels = controlKeySources[key].map(function (name) {
          return gamepad.keyLabels[name] || ""
        }).filter(function (label) { return label !== "" })
        if (labels.length > 0)
          result[key] = labels.join("  ")
      }
      return result
    }

    //! Control keys currently deflected or pressed, for the diagram's live highlight.
    readonly property var activeControls: {
      var keys = []
      for (var name in nameToControl) {
        var key = nameToControl[name]
        if (keys.indexOf(key) < 0 && gamepad.isActive(name))
          keys.push(key)
      }
      return keys
    }

    //! Colour, title and hint per capture state. The hint names the action that clears the state.
    readonly property var statusStyles: ({
      "live": ({ color: Material.color(Material.Green, Material.Shade800),
                 title: qsTr("Driving - keyboard captured"),
                 hint: qsTr("Space centers the axes, Esc stops everything") }),
      "idle": ({ color: Material.color(Material.Orange, Material.Shade800),
                 title: qsTr("Keyboard not captured"),
                 hint: qsTr("Click anywhere in this panel to drive") }),
      "typing": ({ color: Material.color(Material.Blue, Material.Shade800),
                   title: qsTr("Typing in a text field"),
                   hint: qsTr("Press Enter or click below the toolbar to drive again") }),
      "invalid": ({ color: Material.color(Material.Red, Material.Shade800),
                    title: qsTr("No valid joy topic"),
                    hint: qsTr("Nothing is being published - check the joy topic") }),
      "off": ({ color: root.palette.mid, title: qsTr("Not publishing"),
                hint: qsTr("Press play to start the joy stream") })
    })

    readonly property var status: statusStyles[capture.state]

    //! Live fill behind white text: the driving banner, the active mode and pressed table rows.
    readonly property color liveColor: statusStyles["live"].color

    //! Live colour for lines and text on the panel background, lighter on dark themes.
    readonly property color liveForeground: Material.color(Material.Green,
      root.palette.base.hslLightness < 0.5 ? Material.Shade300 : Material.Shade800)

    //! Text on the status colour: white, except on the neutral "off" colour.
    readonly property color statusTextColor: capture.state === "off" ? root.palette.text : "white"

    function makeRow(name, badge, text) {
      var key = gamepad.keyLabels[name] || ""
      var position = inputPositions[name] || { section: qsTr("Other"), order: 10000 }
      return ({
        name: name || "",
        key: key,
        bound: key !== "",
        control: prettyName(name),
        badge: badge,
        text: text,
        section: position.section,
        order: position.order
      })
    }

    //! "left_stick_up" -> "Left stick up"; "button20" -> "Button 20".
    function prettyName(name) {
      if (!name)
        return "(unnamed)"
      if (name.indexOf("button") === 0 && !isNaN(name.substring(6)))
        return "Button " + name.substring(6)
      return name.charAt(0).toUpperCase() + name.substring(1).replace(/_/g, " ")
    }

    //! The item takes text input. Duck-typed to also catch the text input inside a SpinBox.
    function isTextEntry(item) {
      return !!item && item.selectedText !== undefined && item.cursorPosition !== undefined
    }

    //! Hold a synthesized config-switch press for a few publish ticks.
    function pressSwitchButton(name) {
      switchButton = name
      switchTicksLeft = 3
      gamepad.setButton(name, true)
    }

    function describeFailure(config, reason, detail) {
      if (reason === "timeout")
        return qsTr("Switch to %1 timed out - the robot is still in %2.")
               .arg(config).arg(detail || qsTr("an unknown profile"))
      if (reason === "unpressable")
        return qsTr("Cannot switch to %1: it is bound to %2, which this virtual gamepad cannot press.")
               .arg(config).arg(prettyName(detail))
      return qsTr("No gamepad button switches to %1.").arg(config)
    }

    //! Publish the current state once. Also sends the neutral message when the stream stops.
    function publishState() {
      publisher.publish({
        "header": { "stamp": Ros2.now(), "frame_id": "" },
        "axes": gamepad.joyAxes(),
        "buttons": gamepad.joyButtons()
      })
    }

    function publish() {
      if (!publisher || !context.enabled)
        return
      publishState()
      if (switchTicksLeft > 0 && --switchTicksLeft === 0)
        gamepad.setButton(switchButton, false)
    }

  }

  ProfileSwitcher {
    id: switcher

    configSwitches: d.mapping ? d.mapping.config_switches : []
    pressableButtons: gamepad.buttonIndices
    activeProfile: d.activeProfile

    onPressRequested: name => d.pressSwitchButton(name)
    onSucceeded: config => toastManager.show(qsTr("Switched to %1").arg(config))
    onFailed: (config, reason, detail) =>
      toastManager.show(d.describeFailure(config, reason, detail),
                        reason === "timeout" ? "warning" : "error")
  }

  ToastManager {
    id: toastManager
    z: 100
  }

  Dialog {
    id: settingsDialog
    objectName: "settingsDialog"
    anchors.centerIn: parent
    width: Math.min(parent.width * 0.8, 400)
    title: qsTr("Virtual Gamepad Settings")
    standardButtons: Dialog.Ok
    modal: true
    // The panel's controls never take the focus, so hand it back to the gamepad explicitly.
    onClosed: keyFocus.forceActiveFocus()

    GridLayout {
      anchors.fill: parent
      columns: 2
      columnSpacing: 8
      rowSpacing: 8

      Label {
        text: qsTr("Joy topic:")
      }
      TextField {
        id: joyTopicField
        Layout.fillWidth: true
        objectName: "joyTopicField"
        // Editing the topic by hand pins it, e.g. to publish through the joy_satellite.
        Component.onCompleted: text = context.joyTopic ?? "/joy"
        onTextEdited: {
          context.joyTopic = text
          context.joyTopicPinned = true
        }
        onAccepted: settingsDialog.accept()
      }

      Label {
        text: qsTr("Rate (Hz):")
      }
      SpinBox {
        // On creation the SpinBox clamps its initial 0 up to `from`; that must not be stored.
        property bool restored: false

        editable: true
        from: 1
        to: 100
        objectName: "rateSpinBox"
        Component.onCompleted: {
          value = context.rate ?? 30
          restored = true
        }
        onValueChanged: if (restored) context.rate = value
      }

      CheckBox {
        Layout.columnSpan: 2
        objectName: "stickyCheckBox"
        checked: true
        Component.onCompleted: checked = context.sticky ?? true
        text: qsTr("Sticky axes")
        ToolTip.delay: 500
        ToolTip.visible: hovered
        ToolTip.text: qsTr("Axes hold their value between key presses. Off: held key = full deflection.")
        onCheckedChanged: {
          context.sticky = checked
          gamepad.resetAxes()
        }
      }

      Label {
        text: qsTr("Step:")
      }
      DecimalSpinBox {
        property bool restored: false

        objectName: "stepSpinBox"
        decimals: 2
        from: 0.01
        to: 1.0
        stepSize: 0.05
        Component.onCompleted: {
          value = context.step ?? 0.1
          restored = true
        }
        onValueChanged: if (restored) context.step = value
      }
    }
  }

  CaptureState {
    id: capture

    publishing: !!context.enabled
    topicValid: d.publisher !== null
    // True while any element of the panel has the focus.
    panelFocused: captureScope.activeFocus
    windowActive: root.Window.active
    typing: d.isTextEntry(root.Window.activeFocusItem)

    // Keys held when capture ends never report their release.
    onCapturingChanged: if (!capturing) gamepad.reset()
  }

  VirtualGamepadState {
    id: gamepad
    step: context.step ?? 0.1
    sticky: context.sticky ?? true
    // The manager reads the axes again as soon as the switch is released, possibly before the new
    // profile arrives here, so a held deflection is cleared on the press.
    axisResetButtons: switcher.switchAwayButtons
  }

  Component.onCompleted: {
    if (context.joyTopic === undefined)
      context.joyTopic = "/joy"
    if (context.rate === undefined)
      context.rate = 30
    mappingTopicSelect.refresh()
    mappingTopicSelect.restored = true
  }

  Timer {
    interval: context.rate > 0 ? 1000 / context.rate : 1000
    repeat: true
    running: capture.streaming
    onTriggered: d.publish()
  }

  Subscription {
    topic: context.mappingTopic || ""
    messageType: "hector_gamepad_manager_msgs/msg/GamepadMapping"
    qos: Ros2.QoS().transient_local().reliable().keep_last(1)
    onNewMessage: message => d.mapping = d.parseMapping(message)
  }

  Subscription {
    topic: d.profileTopic
    messageType: "std_msgs/msg/String"
    qos: Ros2.QoS().transient_local().reliable().keep_last(1)
    onNewMessage: message => d.activeProfile = message.data
  }

  // Key events bubble up to this scope from whichever element has the focus. Controls use
  // Qt.NoFocus so Space and the arrow keys never activate them; only the text fields take focus.
  FocusScope {
    id: captureScope
    anchors.fill: parent

    Keys.onPressed: event => {
      if (!capture.capturing)
        return
      event.accepted = gamepad.pressKey(event.key, event.modifiers, event.isAutoRepeat)
    }
    Keys.onReleased: event => {
      if (!capture.capturing)
        return
      event.accepted = gamepad.releaseKey(event.key, event.isAutoRepeat)
    }

    // Holds the focus while no text field has it, since the controls never take it.
    Item {
      id: keyFocus
      objectName: "keyFocus"
      focus: true
      activeFocusOnTab: true
    }

    ColumnLayout {
      anchors.fill: parent
      anchors.margins: 8
      spacing: 8

      RowLayout {
        Layout.fillWidth: true

        FuzzySelector {
          id: mappingTopicSelect

          //! Set after the stored topic is restored, so restoring does not count as a retarget.
          property bool restored: false

          function refresh() {
            var topics = Ros2.queryTopics("hector_gamepad_manager_msgs/msg/GamepadMapping")
            topics.sort()
            model = topics
            // Pick the only robot automatically. With several, the user has to choose.
            if (!context.mappingTopic && topics.length === 1)
              text = topics[0]
          }

          Layout.fillWidth: true
          objectName: "mappingTopicSelector"
          placeholderText: qsTr("Gamepad Mapping Topic")

          Component.onCompleted: text = context.mappingTopic ?? ""
          onTextChanged: {
            if (text === context.mappingTopic)
              return
            // Stop the stream first, so the previous robot gets its neutral message.
            if (restored)
              enableButton.checked = false
            context.mappingTopic = text
            // Follow the selected robot unless the user pinned a joy topic of their own.
            if (d.robotNamespace && !context.joyTopicPinned) {
              context.joyTopic = d.robotNamespace + "/joy"
              joyTopicField.text = context.joyTopic
            }
          }
        }
        RefreshButton {
          focusPolicy: Qt.NoFocus
          onClicked: {
            animate = true
            mappingTopicSelect.refresh()
            animate = false
          }
        }
        // Streams continuously, neutral when idle, so the manager never holds a stale value.
        IconButton {
          id: enableButton

          //! Set after restoring, so only a click takes the keyboard.
          property bool restored: false

          objectName: "enableButton"
          checkable: true
          focusPolicy: Qt.NoFocus
          text: checked ? IconFont.iconPause : IconFont.iconPlay
          tooltipText: checked ? qsTr("Stop publishing") : qsTr("Start publishing")
          // A binding would loop, since the handler writes context.enabled.
          Component.onCompleted: {
            checked = context.enabled ?? true
            restored = true
          }
          onCheckedChanged: {
            context.enabled = checked
            // Start and stop the stream from rest.
            gamepad.reset()
            // A pending switch press cannot reach the robot without the stream.
            switcher.abort()
            // Take the keyboard, so driving needs no second click.
            if (checked && restored)
              keyFocus.forceActiveFocus()
            if (!checked && d.publisher)
              d.publishState()
          }
        }
        IconButton {
          objectName: "settingsButton"
          focusPolicy: Qt.NoFocus
          text: IconFont.iconSettings
          tooltipText: qsTr("Settings")
          onClicked: settingsDialog.open()
        }
      }

      // Below the toolbar there are no text fields, so any click here hands the keyboard back.
      Item {
        objectName: "workingArea"
        Layout.fillWidth: true
        Layout.fillHeight: true

        // Reacts on press so flicks capture too; its passive grab leaves flicking to the table.
        TapHandler {
          onPressedChanged: if (pressed) keyFocus.forceActiveFocus()
        }

        ColumnLayout {
          anchors.fill: parent
          spacing: 8

          Rectangle {
            Layout.fillWidth: true
            objectName: "statusBanner"
            implicitHeight: statusRow.implicitHeight + 12
            radius: 4
            color: d.status.color

            RowLayout {
              id: statusRow
              anchors.left: parent.left
              anchors.right: parent.right
              anchors.verticalCenter: parent.verticalCenter
              anchors.margins: 8
              spacing: 8

              // Pulses while messages are going out.
              Rectangle {
                Layout.alignment: Qt.AlignVCenter
                width: 10
                height: 10
                radius: 5
                color: d.statusTextColor
                opacity: capture.state === "off" ? 0.4 : 1

                SequentialAnimation on opacity {
                  running: capture.streaming
                  loops: Animation.Infinite
                  alwaysRunToEnd: true
                  NumberAnimation { to: 0.25; duration: 500; easing.type: Easing.InOutQuad }
                  NumberAnimation { to: 1.0; duration: 500; easing.type: Easing.InOutQuad }
                }
              }
              Label {
                font.bold: true
                color: d.statusTextColor
                text: d.status.title
              }
              TruncatedLabel {
                Layout.fillWidth: true
                color: d.statusTextColor
                opacity: 0.9
                text: d.status.hint
              }
              Caption {
                visible: capture.streaming
                color: d.statusTextColor
                text: context.joyTopic + qsTr(" @ %1 Hz").arg(context.rate ?? 30)
              }
            }
          }

          // Axis readouts, one hand per side. Pressed buttons show in the diagram and the table.
          Rectangle {
            Layout.fillWidth: true
            implicitHeight: readouts.implicitHeight + 16
            radius: 4
            color: palette.alternateBase
            border.width: 1
            border.color: palette.mid

            RowLayout {
              id: readouts
              anchors.centerIn: parent
              spacing: 12

              TriggerBar {
                label: qsTr("LT")
                keyHint: gamepad.keyLabels["left_trigger"]
                value: gamepad.leftTrigger
                trackHeight: leftStickPad.width
                contentColor: palette.text
                activeColor: d.liveForeground
              }

              StickPad {
                id: leftStickPad
                label: qsTr("Left stick")
                keyHint: gamepad.keyLabels["left_stick_y"] + "  " + gamepad.keyLabels["left_stick_x"]
                xValue: gamepad.leftStickX
                yValue: gamepad.leftStickY
                contentColor: palette.text
                activeColor: d.liveForeground
              }

              StickPad {
                id: rightStickPad
                // A wider gap between the two hands.
                Layout.leftMargin: 24
                label: qsTr("Right stick")
                keyHint: gamepad.keyLabels["right_stick_y"] + "  " + gamepad.keyLabels["right_stick_x"]
                xValue: gamepad.rightStickX
                yValue: gamepad.rightStickY
                contentColor: palette.text
                activeColor: d.liveForeground
              }

              TriggerBar {
                label: qsTr("RT")
                keyHint: gamepad.keyLabels["right_trigger"]
                value: gamepad.rightTrigger
                trackHeight: rightStickPad.width
                contentColor: palette.text
                activeColor: d.liveForeground
              }
            }
          }

          // One chip per config, the active one filled. Clicking one presses its switch button.
          RowLayout {
            Layout.fillWidth: true
            spacing: 4

            // A RowLayout misses Repeater items created in the same frame as a resize (Qt 6.4), so
            // the chips sit in a Row.
            Row {
              visible: d.configNames.length > 0
              spacing: 4

              Label {
                anchors.verticalCenter: parent.verticalCenter
                text: qsTr("Mode:")
              }

              Repeater {
                model: d.configNames

                delegate: Button {
                  readonly property bool isActive: modelData === d.activeProfile
                  readonly property bool isPending: modelData === d.pendingProfile

                  objectName: "modeChip_" + modelData
                  // The active chip stays enabled: greyed out it would read as unavailable.
                  enabled: capture.streaming && d.pendingProfile === ""
                  focusPolicy: Qt.NoFocus
                  // The dot marks the active chip without relying on colour.
                  highlighted: isActive
                  Material.accent: d.liveColor
                  text: isActive ? "● " + modelData : (isPending ? modelData + " …" : modelData)
                  ToolTip.delay: 500
                  ToolTip.visible: hovered
                  ToolTip.text: isActive ? qsTr("The robot is in this mode")
                               : !capture.publishing ? qsTr("Start publishing to switch modes")
                               : !capture.topicValid ? qsTr("Set a valid joy topic to switch modes")
                               : qsTr("Switch the robot to %1").arg(modelData)
                  onClicked: switcher.request(modelData)
                }
              }
            }

            Item {
              Layout.fillWidth: true
            }

            ButtonGroup {
              id: viewGroup
            }
            Button {
              ButtonGroup.group: viewGroup
              checkable: true
              checked: !diagramButton.checked
              focusPolicy: Qt.NoFocus
              objectName: "tableButton"
              text: qsTr("Table")
              onClicked: context.showDiagram = false
            }
            Button {
              id: diagramButton
              ButtonGroup.group: viewGroup
              checkable: true
              checked: context.showDiagram ?? true
              focusPolicy: Qt.NoFocus
              objectName: "diagramButton"
              text: qsTr("Diagram")
              onClicked: context.showDiagram = true
            }
          }

          // The diagram when it fits, otherwise the table. Both stay sized to this item, so the
          // diagram can tell whether it fits while it is hidden.
          Item {
            Layout.fillWidth: true
            Layout.fillHeight: true

            GamepadView {
              id: diagram
              anchors.fill: parent
              // Also shown without a mapping, to check that key presses arrive.
              visible: diagramButton.checked && fits
              controlLabels: d.controlLabels
              controlKeys: d.controlKeys
              reservedControls: d.reservedControls
              activeControls: d.activeControls
              contentColor: palette.text
              labelBackgroundColor: palette.base
              accentColor: palette.highlight
              activeColor: d.liveForeground
            }

            ColumnLayout {
              id: table

              //! Column widths, shared by the header and the rows. Next to a double press column
              //! the action column stops at actionWidth; in narrow panels both shrink alike.
              readonly property real keyWidth: 70
              readonly property real controlWidth: 144
              readonly property real actionWidth: 280
              readonly property real actionMaximumWidth: d.hasDoublePress ? actionWidth
                                                                          : Number.POSITIVE_INFINITY
              //! Rows stop short of the scroll bar, and the header keeps the same width.
              readonly property real scrollBarWidth: bindingList.ScrollBar.vertical.visible
                                                     ? bindingList.ScrollBar.vertical.width : 0

              anchors.fill: parent
              visible: !diagram.visible
              spacing: 4

              RowLayout {
                Layout.fillWidth: true
                Layout.leftMargin: 8
                Layout.rightMargin: 8 + table.scrollBarWidth
                visible: d.bindings.length > 0
                opacity: 0.6
                spacing: 8

                Label {
                  Layout.preferredWidth: table.keyWidth
                  font.bold: true
                  text: qsTr("Key")
                }
                Label {
                  Layout.preferredWidth: table.controlWidth
                  font.bold: true
                  text: qsTr("Control")
                }
                Label {
                  Layout.fillWidth: true
                  Layout.preferredWidth: table.actionWidth
                  Layout.maximumWidth: table.actionMaximumWidth
                  font.bold: true
                  text: qsTr("Action")
                }
                Label {
                  Layout.fillWidth: true
                  Layout.preferredWidth: table.actionWidth
                  visible: d.hasDoublePress
                  font.bold: true
                  text: qsTr("Double press")
                }
              }

              ListView {
                id: bindingList
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: d.tableRows
                ScrollBar.vertical: ScrollBar {
                  policy: bindingList.contentHeight > bindingList.height ? ScrollBar.AlwaysOn
                                                                         : ScrollBar.AlwaysOff
                }

                // The Flickable takes presses before the working area's handler sees them.
                TapHandler {
                  onPressedChanged: if (pressed) keyFocus.forceActiveFocus()
                }

                section.property: "section"
                section.delegate: Label {
                  required property string section

                  width: bindingList.width
                  leftPadding: 8
                  topPadding: 12
                  bottomPadding: 4
                  font.bold: true
                  text: section
                }

                delegate: Rectangle {
                  id: rowItem

                  readonly property bool active: gamepad.isActive(modelData.name)

                  width: bindingList.width - table.scrollBarWidth
                  height: bindingRow.implicitHeight + 8
                  color: active ? d.liveColor
                       : hover.hovered ? Qt.rgba(palette.highlight.r, palette.highlight.g,
                                                 palette.highlight.b, 0.2)
                       : index % 2 === 0 ? palette.base : palette.alternateBase

                  HoverHandler {
                    id: hover
                  }

                  RowLayout {
                    id: bindingRow
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.leftMargin: 8
                    anchors.rightMargin: 8
                    spacing: 8

                    Label {
                      Layout.preferredWidth: table.keyWidth
                      font.bold: true
                      color: active ? "white" : palette.text
                      // Dims controls the keyboard cannot reach.
                      opacity: modelData.bound ? 1 : 0.6
                      text: modelData.bound ? modelData.key : "-"
                    }
                    RowLayout {
                      // A nested layout takes fillWidth from its label and would grow otherwise.
                      Layout.fillWidth: false
                      Layout.preferredWidth: table.controlWidth
                      spacing: 6

                      GamepadButtonIcon {
                        // Stick axes and directions have a glyph of their own, the rest show
                        // their control's.
                        controlKey: glyphFiles[modelData.name]
                                    ? modelData.name : (d.nameToControl[modelData.name] || "")
                        size: Math.round(controlName.implicitHeight * 1.4)
                        active: rowItem.active
                        activeColor: "white"
                      }
                      TruncatedLabel {
                        id: controlName
                        Layout.fillWidth: true
                        color: active ? "white" : palette.text
                        text: modelData.control
                      }
                    }
                    RowLayout {
                      Layout.fillWidth: true
                      Layout.preferredWidth: table.actionWidth
                      Layout.maximumWidth: table.actionMaximumWidth
                      spacing: 8

                      Caption {
                        visible: modelData.badge !== ""
                        color: active ? "white" : palette.text
                        text: modelData.badge
                      }
                      TruncatedLabel {
                        Layout.fillWidth: true
                        color: active ? "white" : palette.text
                        text: modelData.text
                      }
                    }
                    TruncatedLabel {
                      Layout.fillWidth: true
                      Layout.preferredWidth: table.actionWidth
                      visible: d.hasDoublePress
                      color: active ? "white" : palette.text
                      text: modelData.doublePress
                    }
                  }
                }
              }
            }
          }

          Hint {
            Layout.fillWidth: true
            visible: diagramButton.checked && !diagram.fits
            horizontalAlignment: Text.AlignHCenter
            opacity: 0.85
            text: qsTr("Enlarge the panel to see the diagram.")
          }

          Hint {
            Layout.fillWidth: true
            visible: d.bindings.length === 0
            horizontalAlignment: Text.AlignHCenter
            opacity: 0.85
            text: context.mappingTopic ? qsTr("Waiting for a mapping on ") + context.mappingTopic
                                       : qsTr("Select a gamepad mapping topic to see the bindings.")
          }
        }
      }
    }
  }
}
