This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_velocity_state_sensor:_ VelocityStateSensorClient

The velocity state of the IOP robot is published as ```nav_msgs::Odometry``` and ```geometry_msgs::TwistStamped``` messages.

#### Parameter:

_tf_frame_robot (str_, Default: "base_link")

> TF frame id of the robot. This value is set in frame_id of the message header.

_hz (float_ , Default: 10.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/fkie_iop_ocu_slavelib/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_velocity_state_odom (nav_msgs::Odometry)_

> Publishes the velocity. Only twist part is valid.

_velocity_state_twist (geometry_msgs::TwistStamped)_

> Publishes the velocity.

#### Subscriber:

> None


