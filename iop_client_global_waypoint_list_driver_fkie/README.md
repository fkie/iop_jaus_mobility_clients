This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_client_global_waypoint_list_driver_fkie:_ GlobalWaypointListDriverClient

Forwards a list of global waypoints to IOP compliant robot.

#### Parameter:

_tf_frame_world (str_, Default: "/world")

> TF frame id used in ROS for global coordinates.

_utm_zone (str_, Default: "32U")

> The UTM zone is used for translation of ROS global position coordinates into Lat/Lon coordinates.

_travel_speed (float_, Default: 1.0)

> Initial travel speed used if a waypoint is set. This value can be changed by _cmd_speed_ topic.

_waypoint_tolerance (float_, Default: 1.0)

> currently not used.

_hz (float_ , Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/iop_ocu_slavelib_fkie/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_global_waypoint (nav_msgs::Path)_

> Current waypoint reported by GlobalWaypointDriver or GlobalWaypointListDriver. If no waypoints are active the path is empty.

#### Subscriber:

_cmd_global_path (nav_msgs::Path)_

> Command with list of global waypoints. The coordinates are transformed to the global frame id and converted to Lat/Lon.

_cmd_speed (std_msgs::Float32)_

> Set the current travel speed.
