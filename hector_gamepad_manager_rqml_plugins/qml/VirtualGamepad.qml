import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtQuick.Window
import Ros2
import RQml.Elements

// Drive a hector_gamepad_manager from the keyboard when no gamepad is at hand, and show what the
// active config binds each control to.
//
// The plugin publishes sensor_msgs/Joy exactly as a real joy driver would, so the manager, its
// plugins and the config switching all behave as usual. Pick the robot by its joy_mapping topic;
// the joy topic is derived from it and can be overridden to publish to the operator station's
// local joy topic instead, letting the joy_satellite forward it.
Rectangle {
  id: root

  property var kddockwidgets_min_size: Qt.size(420, 460)

  color: palette.base

  // Frames the panel in the state colour, so the capture state is readable at a glance and from
  // across the room when several panels are open. No frame while not publishing.
  border.width: capture.state === "off" ? 0 : 2
  border.color: d.status.color

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

    //! Config whose bindings are on show. Always the active one - there is no separate "viewing"
    //! state to fall out of sync with the robot. Before the first profile message arrives the
    //! mapping's default is the best guess.
    readonly property string shownProfile: activeProfile || (mapping ? mapping.default_config : "")

    //! Config a switch has been requested for and not yet confirmed, "" when idle.
    readonly property string pendingProfile: switcher.pendingProfile

    //! Remaining publish ticks a synthesized config-switch press is held for. A single tick can be
    //! missed if it coincides with the manager's own callback, so hold it over a few.
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

    //! One row per bound action of the selected config: which key drives it, which gamepad control
    //! that is, and what it does. This is the whole point of the help view - the manager's own
    //! mapping never mentions a keyboard.
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
      rows.sort(function (a, b) {
        if (a.bound !== b.bound)
          return a.bound ? -1 : 1 // controls with no key on the keyboard sink to the bottom
        return a.control < b.control ? -1 : (a.control > b.control ? 1 : 0)
      })
      return rows
    }

    // Canonical input name -> GamepadView control key. Virtual axis buttons and the raw axis both
    // fold onto the glyph of the control they read from, so a stick bound as an axis and as four
    // directions reads as one control on the diagram.
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

    //! controlLabels for GamepadView, built from the same rows the table shows. Each row carries
    //! the key of the input it actually came from, so a stick bound as two axes shows "W / S" on
    //! its drive row and "A / D" on its steer row rather than one combined cap for both. Rows
    //! whose control has no place on the artwork are simply left out; the table still lists them.
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
            return // an axis bound as both axis and virtual buttons would repeat itself
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

    // Which canonical input each diagram control takes its keycap from when no binding row names a
    // key of its own - the unbound case, where the diagram still shows the layout. A stick names
    // both of its axes; everything else is a single input.
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

    //! Control key -> keyboard keycap text for the diagram, so it carries the same key column the
    //! bindings table does.
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

    //! Colour and wording per capture state, in one table so the two cannot drift apart. The hint
    //! names the single action that clears the state. The colours are semantic rather than palette
    //! roles: they have to read the same way in every theme.
    readonly property var statusStyles: ({
      "live": ({ color: "#2E7D32", title: qsTr("Driving - keyboard captured"),
                 hint: qsTr("Space centers the axes, Esc stops everything") }),
      "idle": ({ color: "#EF6C00", title: qsTr("Keyboard not captured"),
                 hint: qsTr("Click anywhere in this panel to drive") }),
      "typing": ({ color: "#1565C0", title: qsTr("Typing in a text field"),
                   hint: qsTr("Press Enter or click below the toolbar to drive again") }),
      "invalid": ({ color: "#C62828", title: qsTr("No valid joy topic"),
                    hint: qsTr("Nothing is being published - check the joy topic") }),
      "off": ({ color: root.palette.mid, title: qsTr("Not publishing"),
                hint: qsTr("Press Enable to start the joy stream") })
    })

    readonly property var status: statusStyles[capture.state]

    //! The colour that means "this is live", shared by the banner, the panel frame, the mode chip,
    //! the read-outs, the diagram's pressed control and the active binding row. Nothing else in the
    //! panel uses it; palette.highlight covers selection and meta, such as the reserved
    //! config-switch glyphs.
    readonly property color liveColor: statusStyles["live"].color

    //! White carries every state colour; only the unarmed grey needs the theme's own text colour.
    readonly property color statusTextColor: capture.state === "off" ? root.palette.text : "white"

    function makeRow(name, badge, text) {
      var key = gamepad.keyLabels[name] || ""
      return ({
        name: name || "",
        key: key,
        bound: key !== "",
        control: prettyName(name),
        badge: badge,
        text: text
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

    //! Does this item want the keyboard for itself? Duck-typed rather than compared against
    //! TextInput/TextEdit so it covers whatever a Controls style puts inside a SpinBox too.
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

    //! One Joy message carrying the current state. The neutral message that closes the stream is
    //! the same message with everything at rest.
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
    // The manager republishes the profile on every switch, including ones made on a real gamepad,
    // so this both confirms our request and keeps the chips honest about someone else's change.
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


  CaptureState {
    id: capture

    publishing: !!context.enabled
    topicValid: d.publisher !== null
    // The scope has active focus whenever any of its descendants does, so any element counts.
    panelFocused: captureScope.activeFocus
    // Key releases are only delivered to the active window, so a deflection held across an
    // alt-tab would never be released.
    windowActive: root.Window.active
    typing: d.isTextEntry(root.Window.activeFocusItem)

    // Keys held when capture ends never report their release, whatever ended it.
    onCapturingChanged: if (!capturing) gamepad.reset()
  }

  VirtualGamepadState {
    id: gamepad
    step: context.step ?? 0.1
    sticky: context.sticky ?? true
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
    // shownProfile falls back to the mapping's default until a profile message arrives, so
    // nothing needs seeding here.
    onNewMessage: message => d.mapping = d.parseMapping(message)
  }

  Subscription {
    topic: d.profileTopic
    messageType: "std_msgs/msg/String"
    qos: Ros2.QoS().transient_local().reliable().keep_last(1)
    onNewMessage: message => d.activeProfile = message.data
  }

  // The whole plugin is the capture area: key events bubble up from whichever element holds the
  // focus to this scope, so only leaving the panel costs the keyboard. Every control is declared
  // focusPolicy: Qt.NoFocus, which keeps the focus on keyFocus when one is clicked and stops Space
  // and the arrows from being taken as button activation and focus navigation. The text fields are
  // the exception: they take the focus normally, and capture reads "typing" while they hold it.
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

    // Something inside the scope has to own the focus for the scope to have it, and it must not be
    // one of the controls. This empty item takes the role; a click anywhere in the working area
    // hands the keyboard back to it.
    Item {
      id: keyFocus
      objectName: "keyFocus"
      focus: true
      activeFocusOnTab: true
    }

    ColumnLayout {
      anchors.fill: parent
      anchors.margins: 8
      spacing: 6

      RowLayout {
        Layout.fillWidth: true

        FuzzySelector {
          id: mappingTopicSelect

          //! Set once the stored topic and the auto-selection are in, so neither counts as a
          //! retarget.
          property bool restored: false

          function refresh() {
            var topics = Ros2.queryTopics("hector_gamepad_manager_msgs/msg/GamepadMapping")
            topics.sort()
            model = topics
            // A single robot is not a choice, and the panel shows nothing until one is picked.
            // Two or more stays empty: choosing could point the keyboard at the wrong robot.
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
            // Retargeting stops the stream before the topics change, so the robot being left gets
            // its neutral message on context.joyTopic, which still points at it here.
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
          onClicked: mappingTopicSelect.refresh()
        }
      }

      RowLayout {
        Layout.fillWidth: true

        Label {
          text: qsTr("Joy topic:")
        }
        TextField {
          id: joyTopicField
          Layout.fillWidth: true
          objectName: "joyTopicField"
          // Set by hand to the operator station's local joy topic to publish through the
          // joy_satellite instead of straight to the robot. Only a hand-edit pins it; the field is
          // assigned imperatively so selecting a robot can keep steering it.
          Component.onCompleted: text = context.joyTopic ?? "/joy"
          onTextEdited: {
            context.joyTopic = text
            context.joyTopicPinned = true
          }
          // Enter means "done here", which for this panel means giving the keyboard back.
          onAccepted: keyFocus.forceActiveFocus()
        }
        Label {
          text: qsTr("Rate:")
        }
        SpinBox {
          // A SpinBox starts at 0 and clamps itself up to `from` during creation. Writing that back
          // to the context before Component.onCompleted reads it would pin the rate at 1 Hz for
          // good, so the handler stays quiet until the stored value has been restored.
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
      }

      // Publishing and focus are separate on purpose. Enabling starts the stream, which keeps the
      // manager fed with a neutral gamepad even when nothing is pressed - a stream that simply
      // ended would leave the manager holding the last value it saw. Keys only drive while this
      // panel has the keyboard, which is where an unnoticed focus loss would otherwise silently
      // strand a deflection.
      RowLayout {
        Layout.fillWidth: true

        Button {
          id: enableButton

          //! Set once the stored state is in: only a deliberate press takes the keyboard, not a
          //! saved layout being restored.
          property bool restored: false

          objectName: "enableButton"
          checkable: true
          focusPolicy: Qt.NoFocus
          // Assigned once, not bound: the handler writes context.enabled, so a binding on it would
          // loop. The two toggles beside it are the same. A fresh panel opens publishing, since
          // opening the plugin is already the decision to drive.
          Component.onCompleted: {
            checked = context.enabled ?? true
            restored = true
          }
          text: checked ? qsTr("Publishing") : qsTr("Enable")
          onCheckedChanged: {
            context.enabled = checked
            // Never hand over the last deflection when the stream stops or starts.
            gamepad.reset()
            // A synthesized press only reaches the robot while the stream runs, so a request in
            // flight here would sit until it timed out for the wrong reason.
            switcher.abort()
            // Take the keyboard on a deliberate press, so driving does not need a second click.
            if (checked && restored)
              keyFocus.forceActiveFocus()
            if (!checked && d.publisher)
              d.publishState()
          }
        }
        CheckBox {
          objectName: "stickyCheckBox"
          checked: true
          focusPolicy: Qt.NoFocus
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
        // The diagram needs a lot of room, so it replaces the table rather than sharing with it.
        // It is the default: it reads at a glance and draws the layout even before a mapping
        // arrives, which is also why this toggle lives here rather than next to the profile
        // switcher - it stays reachable before a mapping topic has been picked. The table is the
        // fallback for the full list, including the controls the diagram has no place for.
        Button {
          id: diagramButton
          objectName: "diagramButton"
          checkable: true
          focusPolicy: Qt.NoFocus
          Component.onCompleted: checked = context.showDiagram ?? true
          text: checked ? qsTr("Table") : qsTr("Diagram")
          ToolTip.delay: 500
          ToolTip.visible: hovered
          ToolTip.text: qsTr("Switch between the bindings table and the controller diagram")
          onCheckedChanged: context.showDiagram = checked
        }
        Item {
          Layout.fillWidth: true
        }
        // Only worth saying when there are no mode chips to say it instead.
        Label {
          visible: d.configNames.length === 0
          text: qsTr("No profile")
          font.bold: true
        }
      }

      // Everything below the toolbar is safe to click for focus: it holds no text field, so a
      // press anywhere in it can hand the keyboard back without a rule about where it landed.
      Item {
        objectName: "workingArea"
        Layout.fillWidth: true
        Layout.fillHeight: true

        // On press rather than on tap, so a press that turns into a flick still captures the
        // keyboard. DragThreshold (the default) keeps this to a passive grab, which leaves the
        // table's flicking untouched.
        TapHandler {
          onPressedChanged: if (pressed) keyFocus.forceActiveFocus()
        }

        ColumnLayout {
          anchors.fill: parent
          spacing: 6

          // Answers "are my keys reaching the robot?" in one colour, and when they are not, names
          // the reason and the action that clears it.
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

              // Beats while the stream is actually going out, so a dead publisher cannot look
              // like a live one.
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
              // Where the stream is going, beside the state that describes it.
              Caption {
                visible: capture.streaming
                color: d.statusTextColor
                text: context.joyTopic + qsTr(" @ %1 Hz").arg(context.rate ?? 30)
              }
            }
          }

          // Live state, shown the way the controls move rather than as eight numbers. Button state
          // is deliberately not repeated here: the diagram tints a pressed control and the table
          // highlights its row, so both views already show it.
          Rectangle {
            Layout.fillWidth: true
            implicitHeight: readouts.implicitHeight + 16
            radius: 4
            color: palette.alternateBase
            border.width: 2
            // Same colour as the banner, so the box watched while driving carries the state too.
            border.color: d.status.color

            RowLayout {
              id: readouts
              anchors.centerIn: parent
              spacing: 16

              StickPad {
                label: qsTr("Left stick")
                keyHint: gamepad.keyLabels["left_stick_y"] + "  " + gamepad.keyLabels["left_stick_x"]
                xValue: gamepad.leftStickX
                yValue: gamepad.leftStickY
                deadzone: gamepad.axisDeadzone
                contentColor: palette.text
                activeColor: d.liveColor
              }

              ColumnLayout {
                spacing: 6
                TriggerBar {
                  Layout.fillWidth: true
                  label: qsTr("LT")
                  keyHint: gamepad.keyLabels["left_trigger"]
                  value: gamepad.leftTrigger
                  deadzone: gamepad.axisDeadzone
                  contentColor: palette.text
                  activeColor: d.liveColor
                }
                TriggerBar {
                  Layout.fillWidth: true
                  label: qsTr("RT")
                  keyHint: gamepad.keyLabels["right_trigger"]
                  value: gamepad.rightTrigger
                  deadzone: gamepad.axisDeadzone
                  contentColor: palette.text
                  activeColor: d.liveColor
                }
              }

              StickPad {
                label: qsTr("Right stick")
                keyHint: gamepad.keyLabels["right_stick_y"] + "  " + gamepad.keyLabels["right_stick_x"]
                xValue: gamepad.rightStickX
                yValue: gamepad.rightStickY
                deadzone: gamepad.axisDeadzone
                contentColor: palette.text
                activeColor: d.liveColor
              }

              StickPad {
                label: qsTr("D-pad")
                keyHint: gamepad.keyLabels["dpad_up"] + " / " + gamepad.keyLabels["dpad_down"]
                         + "  " + gamepad.keyLabels["dpad_left"] + " / " + gamepad.keyLabels["dpad_right"]
                // Four real buttons rather than an axis pair, so the dot only ever sits on one of
                // nine spots and there is no deadzone to draw.
                discrete: true
                xValue: (gamepad.isButtonPressed("dpad_left") ? 1 : 0)
                        - (gamepad.isButtonPressed("dpad_right") ? 1 : 0)
                yValue: (gamepad.isButtonPressed("dpad_up") ? 1 : 0)
                        - (gamepad.isButtonPressed("dpad_down") ? 1 : 0)
                deadzone: gamepad.axisDeadzone
                contentColor: palette.text
                activeColor: d.liveColor
              }
            }
          }

          // Mode chips. One per config, the active one filled: this is both the indicator of what
          // the robot is in and the control that changes it, so the two can never disagree.
          // Clicking a chip presses that config's reserved switch button; the bindings below always
          // follow whatever the robot reports, including switches made on a real gamepad.
          RowLayout {
            Layout.fillWidth: true
            visible: d.configNames.length > 0
            spacing: 4

            Label {
              text: qsTr("Mode:")
            }

            Repeater {
              model: d.configNames

              delegate: Button {
                readonly property bool isActive: modelData === d.activeProfile
                readonly property bool isPending: modelData === d.pendingProfile

                objectName: "modeChip_" + modelData
                // The active mode stays enabled: a greyed chip reads as unavailable, which is the
                // opposite of what it means here, and pressing it is harmless - the switcher drops
                // a request for the profile already active. A switch is a synthesized button
                // press, so it needs a stream to travel on, not just the Enable button.
                enabled: capture.streaming && d.pendingProfile === ""
                focusPolicy: Qt.NoFocus
                // Filled in the live colour, with the dot repeating it for anyone who cannot rely
                // on colour.
                highlighted: isActive
                Material.accent: d.liveColor
                text: isActive ? "● " + modelData : (isPending ? modelData + " …" : modelData)
                ToolTip.delay: 500
                ToolTip.visible: hovered
                ToolTip.text: isActive ? qsTr("The robot is in this mode")
                             : !capture.publishing ? qsTr("Enable publishing to switch modes")
                             : !capture.topicValid ? qsTr("Set a valid joy topic to switch modes")
                             : qsTr("Switch the robot to %1").arg(modelData)
                onClicked: switcher.request(modelData)
              }
            }

            Item {
              Layout.fillWidth: true
            }
          }

          GamepadView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            // Shown even with no mapping: an unlabelled controller still reflects live input,
            // which is the quickest way to check that the keyboard is getting through.
            visible: diagramButton.checked
            controlLabels: d.controlLabels
            controlKeys: d.controlKeys
            reservedControls: d.reservedControls
            activeControls: d.activeControls
            contentColor: palette.text
            labelBackgroundColor: palette.base
            // accentColor marks reserved config-switch glyphs and modifier badges, not live input.
            accentColor: palette.highlight
            activeColor: d.liveColor
          }

          ListView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            visible: !diagramButton.checked
            clip: true
            model: d.bindings
            ScrollBar.vertical: ScrollBar {}

            // A Flickable consumes the press before an ancestor's handler sees it, so the table
            // needs its own copy of the working area's click-to-capture. Flicking still works:
            // this only takes a passive grab.
            TapHandler {
              onPressedChanged: if (pressed) keyFocus.forceActiveFocus()
            }

            delegate: Rectangle {
              readonly property bool active: gamepad.isActive(modelData.name)

              width: ListView.view.width
              height: bindingRow.implicitHeight + 4
              color: active ? d.liveColor : "transparent"

              RowLayout {
                id: bindingRow
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                spacing: 6

                Label {
                  Layout.preferredWidth: 70
                  font.bold: true
                  color: active ? "white" : palette.text
                  // A control the keyboard cannot reach still gets a row, just without a key.
                  // Dimmed by opacity: palette.mid is a frame colour and reads at about 2:1 on
                  // palette.base.
                  opacity: modelData.bound ? 1 : 0.6
                  text: modelData.bound ? modelData.key : "-"
                }
                // A clipped control name or description can be read in full by hovering it.
                TruncatedLabel {
                  Layout.preferredWidth: 120
                  color: active ? "white" : palette.text
                  text: modelData.control
                }
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
            }
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
