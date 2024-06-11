# ROS Node for 3D coils

Attempt at adapting [3d coils backend](https://github.com/vfrancescon/coil_libs) into a ROS node in my spare time.

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

* PSU nodes have a [Voltage/Current interface](msg/VI.msg), with embedded calls to polarity setting.

* Each PSU node has a ROS Param "debug", to be used to omit all Serial calls to PSU.

* V/I limits are set at 70% of each supply's rated values. Those can be set as ROS Params.
See [6PSU + Field](launch/6psu_field.launch) example for usage.

* The middleware node is spun, with a [Magnetic Field Interface](msg/magField.msg), which is automatically translated to input msgs for each PSU node.

* The [Field Node](src/Field_node.cpp), takes ROS Params x/y/z/Num and x/y/z/Root.
  * x/y/z/Num is an integer (1 or 2) that specifies how many supplies handle the given axis.
  * x/y/z/Root is a string that specifies the address of the first supply.

> **Example**
>
> xNum = 2; xRoot = "/PSU0". The X component of the "/Field" topic will be bound to "/PSU0", "/PSU1".
>
> See [6PSU + Field](launch/6psu_field.launch) example for usage.

* The Z2 PSU is unique in that it lacks a Polarity interface, that is normal and handled internally.

* New values are published only if they differ from the currently outputted values. This is to prevent the supplies from being overwhelmed.

* Current Max frequency is unknown, but bound by X2 not getting overwhelmed by just existing. Needs investigation.

## Dependencies

[3d coils backend](https://github.com/vfrancescon/coil_manipulator)

## Utilities

### General Notes

If the PSUs are ever requested to push more voltage/current than they can handle (70% of the rated values), the field interface will shutdown. In this case, it is best you close ros nodes, check how the shutdown was triggered and then start again.

If the change in field magnitude is ever bigger than 15mT, the field interface will also shutdown. Similarly, there is a limit of 22/22/22mT on each axis.

### Manual Field Input

[Launch File](launch/field_manual_op.launch). This program starts up the Field interface and takes in field inputs(e.g. bx ENTER -> by ENTER -> bz ENTER), one at a time. To exit, type "q" or "shutdown", then close the field interface with Ctrl+C as usual.

To launch this, use:

```bash
roslaunch ros_coils field_manual_op.launch
```

### Manual sweep

[Launch File](launch/sweep_man.launch). This program starts up the Field interface and sweeps between the negative and positive sides of the argument "MaxAbs". This argument can be edited in the launch file. While technically multiple axis can be swept at once, they will not be synchronous unless their respective magnitude is equal.

Absolute magnitude, frequency and increment size are editable from [here](launch/sweep_man.launch).

To launch this, use:

```bash
roslaunch ros_coils sweep_man.launch
```

### CSV Sweep

[Launch File](launch/sweep_csv.launch). This program starts up the Field interface and runs through a csv at a given frequency. The template for csv files is given [here](csv/sample_csv.csv). Please be mindful this checks that all inputs in the csv are valid numbers, but all safety checks are still handled top-level by the field and PSU interfaces.

Csv file path (relative to the package) and frequency can be edited [here](launch/sweep_csv.launch).

To launch, use:

```bash
roslaunch ros_coils sweep_csv.launch
```

## Author

Vittorio Francescon
