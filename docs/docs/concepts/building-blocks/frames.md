---
sidebar_position: 3
title: Frames
---

# Frames

The page on [TF in ROS](../ros-concepts/tf) discusses the importance of spatial transforms for robotic applications.
TF can be thought of as a live database that keeps track of coordinate frames and their relationships over time. The AICA
System leverages this framework internally and facilitates interaction with TF. In particular, user-defined
frames are directly included in an AICA application and are available to all components at runtime. These so-called
**application frames** can be created and modified in the 3D scene view, and their poses will be updated in real time.

:::tip

[This guide](/core/examples/guides/application-frames) contains an in-depth review of methods to create, edit, and
record application frames in AICA Studio.

:::

## Using frames as signals

In data-flow programming, it might often be necessary to extract the pose of one specific frame from TF in real time and
publish that information as a continuous signal.

For instance, a vision component might publish the pose of a detected object to TF, while a motion generator component
requires the target pose to be received as a signal. Ideally, the vision component would output the pose as a signal,
but this is not always the case. For these situations, the AICA Core provides components that can extract the pose of a
desired frame from TF and publish it as a signal, or conversely, receive a pose from a signal and send it to TF.

### Frame to Signal

The Frame to Signal component looks up a desired frame at the specified rate and publishes it on its output as a
Cartesian pose. The component has several additional parameters to configure its behavior:

- **Frame**: defines which frame should be looked up from TF
- **Reference frame** (optional): defines in what reference frame the frame should be expressed in, default is `world`
- **Remap frame** (optional): if provided, the name of the Cartesian pose output will be set to this value
- **Remap reference frame** (optional): if provided the reference frame of the Cartesian pose output will be set to this
  value

### Signal to Frame

The Signal to Frame component receives a Cartesian pose on its input and sends the information to TF at the specified
rate. The component has two additional parameters to configure its behavior:

- **Remap frame** (optional): if provided, the name of the transform sent to TF will be set to this value
- **Remap reference frame** (optional): if provided the reference frame of the transform sent to TF will be set to this
  value
<!-- 
:::tip

See [this page](../../examples/core-components/signals-tf.md) for examples using these components in AICA Studio.
TODO: link the examples section of the docs

::: -->