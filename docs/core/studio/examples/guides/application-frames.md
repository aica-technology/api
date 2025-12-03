---
sidebar_position: 4
title: Application frames
---

import createFrame from './assets/create-frame.webm'
import recordFrame from './assets/record-frame.webm'
import editFrame from './assets/edit-frame.webm'

# Application Frames in AICA Studio

This guide explains the various methods to create and modify application frames in AICA Studio.

:::tip

If you haven't done so already, review the corresponding [concepts page](/docs/concepts/building-blocks/frames) first.

:::

There are two ways to access settings and functionality for application frames in AICA Studio, depending on where the 3D
scene lies:

- With the 3D scene on the main view of the AICA Studio editor, the options to create and record frames can be found
  under the **Scene** tab in the right panel.
- With the 3D scene in the right panel, under **3D view**, the options can be found by clicking on the **Settings**
  button on the top left of the scene.

## Create a frame 

The "Create a frame" option can be used to create a new named frame, which can then be dragged to the desired location
in the scene. The position, orientation and reference frame of the frame appear under `frames` in the application YAML
and are updated on drag.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={createFrame} type="video/webm" />
    Create a frame.
  </video>
</div>
<br />

:::note

Frames are created at the origin of the scene and have reference frame `world` by default.

:::

## Record a frame 

Application frames can be created by recording an existing frame from TF. The "Record a frame" option allows to save any
available frame from TF under a new name in a running application. This can be useful to obtain the end-effector pose of
a robot in various locations, which will then be used as waypoints for the robot movements in the application flow. In
the example below, the end-effector frame `tool0` of the robot is recorded as `target`.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={recordFrame} type="video/webm" />
    Record a frame.
  </video>
</div>
<br />

:::note

Frames are recorded in reference frame `world` by default. In the future, recording frames in configurable reference
frames might be supported.

:::

## Edit frames in application YAML

Users that are familiar with the application YAML can also add, remove, and edit frames in the application YAML.

<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={editFrame} type="video/webm" />
    Edit a frame.
  </video>
</div>

<!-- TODO: add examples here -->
