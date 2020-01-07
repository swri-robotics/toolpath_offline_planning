std_msgs/Header header
opp_msgs/ProcessType process_type
geometry_msgs/PoseArray[] paths
uint32 dwell_time                       # Time (in seconds) to dwell after the tool path has been completed
geometry_msgs/Pose tool_offset          # 6-DOF tool_offset
opp_msgs/ToolPathParams params      # Numerical parameters used for tool path generation
