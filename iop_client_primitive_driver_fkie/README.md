This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


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