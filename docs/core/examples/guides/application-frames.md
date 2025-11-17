---
sidebar_position: 4
title: Application frames
---

import createFrame from './assets/create-frame.gif'
import recordFrame from './assets/record-frame.gif'
import editFrame from './assets/edit-frame.gif'

# Application Frames in AICA Studio

This guide explains the various methods to create and modify application frames in AICA Studio.

:::tip

If you haven't done so already, review the corresponding [concepts page](/docs/concepts/building-blocks/frames) first.

:::

## Create frame in 3D scene view

With the 3D scene on the main view of the AICA Studio editor, the "Create a frame" option in the Settings section of the
Scene tab on the right panel, can be used to create a new named frame which can then be dragged to the desired location
in the scene. The position, orientation and reference frame of the frame appear under `frames` in the application YAML
and are updated on drag.

<div class="text--center">
  <img src={createFrame} alt="Create a frame" />
</div>

:::note

1. Frames are created at the origin of the scene and have reference frame `world` by default.

2. Frames can also be created with the 3D view in the right panel. The option can be found under the `Settings` menu on
   the top left of the scene.

:::

## Record frame in 3D scene view

Application frames can be created by recording an existing frame from TF. The "Record a frame" option allows to save any
available frame from TF under a new name in a running application. This can be useful to obtain the end-effector pose of
a robot in various locations, which will then be used as waypoints for the robot movements in the application flow. In
the example below, the end-effector frame `tool0` of the robot is recorded as `target`.

<div class="text--center">
  <img src={recordFrame} alt="Record a frame" />
</div>

<details>
  <summary>Application YAML</summary>

    ```yaml
    schema: 2-0-6
    dependencies:
      core: v5.0.0
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
        on_start:
          x: 0
          y: -20
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

## Edit frames in application YAML

Users that are familiar with the application YAML can also add, remove, and edit frames in the application YAML.

<div class="text--center">
  <img src={editFrame} alt="Edit a frame" />
</div>

<!-- TODO: add examples here -->
