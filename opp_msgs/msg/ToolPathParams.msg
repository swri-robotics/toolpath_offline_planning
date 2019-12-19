# ToolPathParams.msg
# This message is meant to facilitate a backwards-compatible change to
# record the tool path parameters used in the creation of a particular
# tool path.  It gathers up various numerical information specified when
# creating the path and saves it in a single structure, insulating the
# ToolPath message from being affected by changes in logged data.

# The general Noether configuration parameters
noether_msgs/ToolPathConfig config

# The segmentation parameters
float64 curvature_threshold
int32 min_polygons_per_cluster
