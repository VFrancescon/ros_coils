# ROS Node for 3D coils

Attempt at adapting [3d coils backend](https://github.com/vfrancescon/coil_manipulator) into a ROS node in my spare time.

## Usage: Remote Host

Follow these points on the host machine connected with the coils. This machine must clear the dependencies section.

1. Clone the main branch in your ros workspace.

2. Build messages and nodes.

```bash
catkin_make
```

3. Make sure the following are in your ~/.bashrc

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
export ROS_IP=192.168.0.10
export ROS_MASTER_URI=http://192.168.0.10:11311
```

4. Run the nodes

```bash
roslaunch ros_coils middleware.launch
```

This spins up all the psus as a separate node each.

## Usage: Local client

Follow these points on the local machine you want to push fields with. This machine does not need to clear the dependencies section.

1. Clone the RemoteMachine branch in your ros workspace.

2. Build messages.

```bash
catkin_make
```

3. Make sure the following are in your ~/.bashrc

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
export ROS_IP=192.168.0.15
export ROS_MASTER_URI=http://192.168.0.10:11311
```

4. You should now be able to publish to the \field topic with no issues.

## Notes

* PSU nodes have a [Voltage/Current interface](msg/VI.msg) and [Polarity interface](msg/Polarity.msg).

* The middleware node is spun, with a [Magnetic Field Interface](msg/magField.msg), which is automatically translated to input msgs for each PSU node.

* The Z2 PSU is unique in that it lacks a Polarity interface, that is normal and handled internally.

* New values are published only if they differ from the currently outputted values. This is to prevent the supplies from being overwhelmed.

* Current Max frequency is unknown, but bound by X2 not getting overwhelmed by just existing. Needs investigation.

## Dependencies

[3d coils backend](https://github.com/vfrancescon/coil_manipulator)

## Author

Vittorio Francescon
