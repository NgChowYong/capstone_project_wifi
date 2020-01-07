# CtrlData.msg
# Version tircgo_1107

# mode the controller is in
int16 mode

# stage bitmap
# stage = stage bitwire OR mode
int16 stage_bm

# vars set by UART
int16 tracking_status
int16[] lidar_levels


# coordinate, not reliable, just a reference
# set to 0 if calibration is triggered
geometry_msgs/Quaternion coor

# runtime info, "nd" means "RoutNode"

## training
RouteNode nd_training
### can be direcly drawn by graphviz
string graph

## working
RouteNode nd_ocp
RouteNode nd_target
RouteNode[] work_list