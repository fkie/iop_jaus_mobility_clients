This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_global_pose_sensor:_ GlobalPoseSensorClient

The global position data of the IOP robot is published as NavSatFix and Imu ROS messages. A TF from world to robot frame will also be broadcasted.

#### Parameter:

_tf_frame_world (str_, Default: "world")

> TF frame id used in ROS for global coordinates.

_tf_frame_anchor (str_, Default: "anchor")

> TF frame id for achor between `world` and `robot` frame. It is usefull since for visualization in Rviz.

_tf_frame_robot (str_, Default: "base_link")

> TF frame id of the robot.

_hz (float_ , Default: 10.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/fkie_iop_ocu_slavelib/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

_anchor_easting (double_, Default: 0.0)

> Default easting coordinate for the anchor. It can be replaced by `fix_anchor` topic.

_anchor_norting (double_, Default: 0.0)

> Default northing coordinate for the anchor. It can be replaced by `fix_anchor` topic.

_anchor_altitude (double_, Default: 0.0)

> Default altitude coordinate for the anchor. It can be replaced by `fix_anchor` topic.

_publish_world_anchor (bool_ Default: true)

> Enables the publishing of /world to anchor tf. You can publish this disable the publishing of this tf by this service and publish tf itself.

#### Publisher:

_fix (sensor_msgs::NavSatFix)_

> Publishes global position.

_imu (sensor_msgs::Imu)_

> Publishes global orientation.

_global_pose (geometry_msgs::PoseStamped)_

> Publishes global pose transfromed from Lat/Lon to UTM. The `frame_id` is set UTM zone.

#### Subscriber:

_fix_anchor (sensor_msgs::NavSatFix)_

> Allows you to add an achor TF between `world` and `robot` frame.

#### Tf:

_tf_frame_world_ -> _tf_frame_robot_

