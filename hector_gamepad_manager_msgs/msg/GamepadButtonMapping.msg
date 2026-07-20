# Mapping of a single button to its actions.
# Buttons 0-10 are physical; 11-24 are virtual (axis deflection past a deadzone).
uint8 index
string plugin       # pluginlib class name, exactly as in YAML
GamepadAction[] actions
