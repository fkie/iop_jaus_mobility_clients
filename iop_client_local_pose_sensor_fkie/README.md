This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_client_local_pose_sensor_fkie:_ LocalPoseSensorClient

The local position received from IOP complient robot is published as ```Tf```, ```geometry_msgs::PoseStamped``` and ```nav_msgs::Odometry```

#### Parameter:

_tf_frame_odom (str_, Default: "odom")

> Defines the odometry frame id. This parameter is only regarded if _source_type_ is *0*.

_tf_frame_robot (str_, Default: "base_link")

> Defines the robot frame id. This parameter is only regarded if _source_type_ is *0*.

_hz (float_, Default: 10.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/iop_ocu_slavelib_fkie/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created. Note: some service reports if a new value is availabe and not if the position is changend. This can lead to very high rates if you decide to use a zero value.

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
