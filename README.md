# ROS Node for 3D coils

Attempt at adapting [3d coils backend](https://github.com/vfrancescon/coil_manipulator) into a ROS node in my spare time.

## Usage

Clone the RemoteBrahch in your ros workspace, build to obtain the messages and go ham.

Sample message:

```bash
rostopic pub /field ros_coils/magField "bx: 10.0
by: 0.0
bz: 0.0" 
```

PSU nodes have a [Voltage/Current interface](msg/VI.msg) and [Polarity interface](msg/Polarity.msg).

Further, a middleware node is spun, with a [Magnetic Field Interface](msg/magField.msg), which is automatically translated to input msgs for each PSU node.

## Notes

* The Z2 PSU is unique in that it lacks a Polarity interface, that is normal and handled internally.

* New values are published only if they differ from the currently outputted values. This is to prevent the supplies from being overwhelmed.

* Current Max frequency is unknown, but bound by X2 not getting overwhelmed by just existing. Needs investigation.

## Dependencies

[3d coils backend](https://github.com/vfrancescon/coil_manipulator). Only required to run the local branch.

## Author

Vittorio Francescon
