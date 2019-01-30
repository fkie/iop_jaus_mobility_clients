This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_local_waypoint_driver:_ LocallWaypointDriverClient

Forwards the local waypoint command to IOP compliant robot.

#### Parameter:

_tf_frame_robot (str_, Default: "base_link")

> TF frame id used in ROS for local coordinates.

_travel_speed (float_, Default: 1.0)

> Initial travel speed used if a waypoint is set. This value can be changed by _cmd_speed_ topic.

_hz (float_ , Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/fkie_iop_ocu_slavelib/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_local_waypath (nav_msgs::Path)_

> Current waypoint reported by LocalWaypointDriver or LocalWaypointListDriver. If no waypoints are active the path is empty.


#### Subscriber:

_cmd_local_pose (geometry_msgs::PoseStamped)_

> Command for local waypoint. The coordinates are transformed to the local frame id.

_cmd_speed (std_msgs::Float32)_

> Set the current travel speed.
