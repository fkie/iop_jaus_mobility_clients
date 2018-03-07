See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.


# Interfaces

The repository contains clients designed to control services on IOP complient robot. All client services are based on ```SlaveHandlerInterface``` and use funtionality of [Slave](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#iop_ocu_slavelib_fkie). 

List of service plugins in this repository:

[iop_client_global_pose_sensor_fkie: GlobalPoseSensorClient](#iop_client_global_pose_sensor_fkie-globalposesensorclient)  
[iop_client_global_waypoint_driver_fkie: GlobalWaypointDriverClient](#iop_client_global_waypoint_driver_fkie-globalwaypointdriverclient)  
[iop_client_global_waypoint_list_driver_fkie: GlobalWaypointListDriverClient](#iop_client_global_waypoint_list_driver_fkie-globalwaypointlistdriverclient)  
[iop_client_local_pose_sensor_fkie: LocalPoseSensorClient](#iop_client_local_pose_sensor_fkie-localposesensorclient)  
[iop_client_primitive_driver_fkie: PrimitiveDriverClient](#iop_client_primitive_driver_fkie-primitivedriverclient)


## _iop_client_global_pose_sensor_fkie:_ GlobalPoseSensorClient

The global position data of the IOP robot is published as NavSatFix and Imu ROS messages. A TF from world to robot frame will also be broadcasted.

#### Parameter:

_tf_frame_world (str_, Default: "world")

> TF frame id used in ROS for global coordinates.

_tf_frame_anchor (str_, Default: "anchor")

> TF frame id for achor between `world` and `robot` frame. It is usefull since for visualization in Rviz.

_tf_frame_robot (str_, Default: "base_link")

> TF frame id of the robot.

_hz (float_ , Default: 10.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

_anchor_easting (str_, Default: 0.0)

> Default easting coordinate for the anchor. It can be replaced by `fix_anchor` topic.

_anchor_norting (str_, Default: 0.0)

> Default northing coordinate for the anchor. It can be replaced by `fix_anchor` topic.


#### Publisher:

_fix (sensor_msgs::NavSatFix)_

> Publishes global position.

_imu (sensor_msgs::Imu)_

> Publishes global orientation.

#### Subscriber:

_fix_anchor (sensor_msgs::NavSatFix)_

> Allows you to add an achor TF between `world` and `robot` frame.

#### Tf:

_tf_frame_world_ -> _tf_frame_robot_


## _iop_client_global_waypoint_driver_fkie:_ GlobalWaypointDriverClient

Forwards the global waypoint command to IOP compliant robot.

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

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_global_waypoint (nav_msgs::Path)_

> Current waypoint reported by GlobalWaypointDriver or GlobalWaypointListDriver. If no waypoints are active the path is empty.


#### Subscriber:

_cmd_pose (geometry_msgs::PoseStamped)_

> Command for global waypoint. The coordinates are transformed to the global frame id and converted to Lat/Lon.

_cmd_speed (std_msgs::Float32)_

> Set the current travel speed.


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

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_global_waypoint (nav_msgs::Path)_

> Current waypoint reported by GlobalWaypointDriver or GlobalWaypointListDriver. If no waypoints are active the path is empty.

#### Subscriber:

_cmd_path (nav_msgs::Path)_

> Command with list of global waypoints. The coordinates are transformed to the global frame id and converted to Lat/Lon.

_cmd_speed (std_msgs::Float32)_

> Set the current travel speed.


## _iop_client_local_pose_sensor_fkie:_ LocalPoseSensorClient

The local position received from IOP complient robot is published as ```Tf```, ```geometry_msgs::PoseStamped``` and ```nav_msgs::Odometry```

#### Parameter:

_tf_frame_odom (str_, Default: "odom")

> Defines the odometry frame id. This parameter is only regarded if _source_type_ is *0*.

_tf_frame_robot (str_, Default: "base_link")

> Defines the robot frame id. This parameter is only regarded if _source_type_ is *0*.

_hz (float_, Default: 10.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created. Note: some service reports if a new value is availabe and not if the position is changend. This can lead to very high rates if you decide to use a zero value.

_send_inverse_trafo (bool_, Default: true)

> Sets the transform direction while publish TF frame. True: tf_frame_robot -> tf_frame_odom

#### Publisher:

_pose (geometry_msgs::PoseStamped)_

> Publishes the local position with tf frame _tf_frame_odom_.

_odom (nav_msgs::Odometry)_

> Publishes the local odometry with tf frame _tf_frame_odom_.

#### Subscriber:

> None

#### Tf:

_tf_frame_robot_ -> _tf_frame_odom_


## _iop_client_primitive_driver_fkie:_ PrimitiveDriverClient

Converts ```geometry_msgs::Twist``` into IOP effort command messages to drive a robot platform. The twist value should be between -1 and 1 to be translated accurate to the effort. For other twist value you can use `max_linear` and `max_angular` parameter to adapt your values.

#### Parameter:

_invert_yaw (bool_, Default: true)

> Inverts the yaw control orientation.

_use_stamped (bool_, Default: true)

> If *true* use _geometry_msgs::TwistStamped_ instead of _geometry_msgs::Twist_ to publish the commands.

_max_linear (double_, default: 1.0)

> The maximal velocity in twist message. Based on this value the received velocity will be scaled to maximal effort of 100 percent.

_max_angular (double_, default: 1.5)

> The maximal angular velocity in twist message. Based on this value the received velocity will be scaled to maximal effort of 100 percent.


#### Publisher:

> None

#### Subscriber:

_joy_cmd_vel (geometry_msgs::TwistStamped)_

> Twist commands for the platform, if _use_stamped_ is set to *true*.

_joy_cmd_vel (geometry_msgs::Twist)_

> Twist commands for the platform, if _use_stamped_ is set to *false*.
