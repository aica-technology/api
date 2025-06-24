---
sidebar_position: 1
title: Signals and TF
---

# Signals and TF

The page on [TF in ROS](../../ros-concepts/tf.md) discusses the importance of TF for robotic applications. TF can be
thought of as a live database that keeps track of coordinate frames and their relationship over time. In data-flow
programming, it might often be necessary to extract the pose of one specific frame from TF in real time and publish that
information as a continuous signal.

For instance, an vision component might publish the pose of a detected object to TF and a motion generator component
requires the target pose to be received as a signal. Ideally, the vision component would output the pose as a signal,
but this is not always given. For these cases, the AICA Core provides components that can extract the pose of a desired
frame from TF and publish it as a signal, or conversly receive a pose from a signal and send it to TF.

## Frame to Signal

The Frame to Signal component looks up a desired frame at the specified rate and publishes it on its output as a
Cartesian pose. The component has several additional parameters to configure its behavior:

- **Frame**: defines which frame should be looked up from TF
- **Reference frame** (optional): defines in what reference frame the frame should be expressed in, default is `world`
- **Remap frame** (optional): if provided, the name of the Cartesian pose output will be set to this value
- **Remap reference frame** (optional): if provided the reference frame of the Cartesian pose output will be set to this
  value

## Signal to Frame

The Signal to Frame component receives a Cartesian pose on its input and sends the information to TF at the specified
rate. The component has two additional parameters to configure its behavior:

- **Remap frame** (optional): if provided, the name of the transform sent to TF will be set to this value
- **Remap reference frame** (optional): if provided the reference frame of the transform sent to TF will be set to this
  value

:::tip

See [this page](../../../examples/core-components/signals-tf.md) for examples using these components in AICA Studio.

:::