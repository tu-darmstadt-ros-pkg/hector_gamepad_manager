# One action bound to a button, for a single input event.
uint8 EVENT_PRESS=0
uint8 EVENT_DOUBLE_PRESS=1
uint8 EVENT_HOLD=2
uint8 EVENT_RELEASE=3

uint8 event
string function     # function name passed to the plugin
string description  # human-readable, from YAML; may be empty
