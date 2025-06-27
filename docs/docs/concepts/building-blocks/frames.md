---
sidebar_position: 3
title: Frames
---

import createFrame from './assets/create-frame.gif'
import recordFrame from './assets/record-frame.gif'
import editFrame from './assets/edit-frame.gif'

# Frames

The page on [TF in ROS](../ros-concepts/tf.md) discusses the importance of spatial transforms for robotic applications.
TF can be thought of as a live database that keeps track of coordinate frames and their relationships over time. The AICA
System leverages this framework internally and facilitates interaction with TF. In particular, user-defined
frames are directly included in an AICA application and are available to all components at runtime. These so-called
**application frames** can be created and modified in the 3D scene view, and their poses will be updated in real time.

## Frames in AICA Studio

This section explains the various methods to create and modify application frames in AICA Studio.

### Create frame in 3D scene view

Using the "Create a frame" dropdown in the 3D scene view in AICA Studio, users can create a new named frame which can
then be dragged to the desired location in the scene. The position, orientation and reference frame of the frame appear
under `frames` in the application YAML and are updated on drag.

<div class="text--center">
  <img src={createFrame} alt="Create a frame" />
</div>

:::note

Frames are created at the origin of the scene and have reference frame `world` by default.

:::

### Record frame in 3D scene view

Application frames can be created by recording an existing frame from TF. The "Record a frame" dropdown allows to save
any available frame from TF under a new name in a running application. This can be useful to obtain the end-effector
pose of a robot in various locations, which will then be used as waypoints for the robot movements in the application
flow. In the example below, the end-effector frame `tool0` of the robot is recorded as `target`.

<div class="text--center">
  <img src={recordFrame} alt="Record a frame" />
</div>

<details>
  <summary>Application YAML</summary>

    ```yaml
    schema: 2-0-4
    dependencies:
      core: v4.4.1
    on_start:
      load:
        hardware: hardware
    hardware:
      hardware:
        display_name: Hardware Interface
        urdf: Generic six-axis robot arm
        rate: 100
        events:
          transitions:
            on_load:
              load:
                controller: robot_state_broadcaster
                hardware: hardware
        controllers:
          robot_state_broadcaster:
            plugin: aica_core_controllers/RobotStateBroadcaster
            events:
              transitions:
                on_load:
                  switch_controllers:
                    hardware: hardware
                    activate: robot_state_broadcaster
    graph:
      positions:
        hardware:
          hardware:
            x: 500
            y: -20
    ```

</details>

:::note

Frames are recorded in reference frame `world` by default. In the future, recording frames in configurable reference
frames might be supported.

:::

### Edit frames in application YAML

Users that are familiar with the application YAML can also add, remove, and edit frames in the application YAML. 

<div class="text--center">
  <img src={editFrame} alt="Edit a frame" />
</div>

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

:::tip

See [this page](../../examples/core-components/signals-tf.md) for examples using these components in AICA Studio.

:::