# No process
uint8 NONE = 0

# Depaint Processes
uint8 PROCESS_PAINT = 1
uint8 PROCESS_DEPAINT = 2

# val - which process has actually been selected
uint8 val

# If you add a new process to this file and it needs to be available in
# offline planner, remember to add it to the drop-down menu in the gui:
# opp_gui/src/widgets/tool_path_parameters_editor_widget.cpp
