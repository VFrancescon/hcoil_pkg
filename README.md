![CI/CD Status](https://github.com/VFrancescon/hcoil_pkg/actions/workflows/ros2_ci.yaml/badge.svg)

# hcoil_pkg

ROS2 bindings for the [Helmholtz coil setup and PSU array](git@github.com:VFrancescon/coil_libs.git).  

## Usage 

The [field](launch/field_launch.py) spins up a field node that subscribes to the "magfield" topic. 
The same node then publishes VI messages to all the PSUs connected, which are configured in the same launch file. The currently applied field can be fetched using the "ComputeField" service.

### Publish field

[Publish field](scripts/publish_field.py) spins up and publishes a single field message. The field to publish can be set with ROS params. See the node for more info.

## General Notes

* PSU nodes have a [Voltage/Current interface](https://github.com/VFrancescon/hcoil_interfaces/blob/main/msg/VoltAmp.msg), with embedded calls to polarity setting.

* Each PSU node has a ROS Param "debug", to be used to omit all Serial calls to PSU.

* V/I limits are set at 70% of each supply's rated values. Those can be set as ROS Params.
See [6PSU + Field](launch/field_launch.py) example for usage.

* The middleware node is spun, with a [Magnetic Field Interface](https://github.com/VFrancescon/hcoil_interfaces/blob/main/msg/MagField.msg), which is automatically translated to input msgs for each PSU node.

* The [Field Node](src/field_node.cpp), takes ROS Params x/y/z/Num and x/y/z/Root.
  * x/y/z/Num is an integer (1 or 2) that specifies how many supplies handle the given axis.
  * x/y/z/Root is a string that specifies the address of the first supply.

> **Example**
>
> xNum = 2; xRoot = "/PSU0". The X component of the "/Field" topic will be bound to "/PSU0", "/PSU1".
>
> See [PSU + Field](launch/field_launch.py) example for usage.

* The Z2 PSU is unique in that it lacks a Polarity interface, that is normal and handled internally.

* New values are published only if they differ from the currently outputted values. This is to prevent the supplies from being overwhelmed.

* Current Max frequency is unknown, but bound by X2 not getting overwhelmed by just existing. Needs investigation.