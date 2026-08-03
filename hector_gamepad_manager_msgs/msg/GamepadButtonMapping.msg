# Mapping of a single button to its actions.
# Buttons 0-31 are physical (read 1:1 from the Joy message); 32-45 are virtual
# (axis deflection past a deadzone, keyed by name in the "axis_buttons" config section).
uint8 index
string plugin       # pluginlib class name, exactly as in YAML
GamepadAction[] actions
