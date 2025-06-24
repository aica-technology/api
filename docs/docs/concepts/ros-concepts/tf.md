---
sidebar_position: 5
title: TF
---

import tfTree from './assets/tf-tree.png'

# TF in ROS

In robotics, different components, such as sensors, actuators, and robot links, often operate in their own local
coordinate frames. One might ask questions like:

- What is the pose of an object detected by the camera relative to the robot base frame?
- Where was the tracked object relative to the world frame 5 seconds ago?
- Where is the end-effector relative to the environmanet?

TF, which stands for **Transform**, is a powerful framework designed to keep track of multiple coordinate frames and
their relationship over time. It provides a standardized way to transform data between these frames, ensuring that all
parts of the system can communicate spatial information accurately.

TF is widely adopted in ROS due to its key features:

- **Frame management**: TF maintains a dynamic tree of coordinate frames, each with a unique name (e.g. `world`,
  `camera`, `tool`). The relationships between these frames are continuously updated as the robot moves or interacts
  with its environment.
- **Transform Queries**: TF allows to query the position and orientation (transform) of any frame relative to another at
  any point in time, given that they are part of the same TF tree. This is essential for tasks like converting sensor
  data to a common reference frame or planning robot motions.
- **Real-time operation**: TF broadcasts and listens to transforms in real time, enabling up-to-date spatial reasoning.
  This is critical for applications such as sensor fusion, motion planning, and real-time visualization.
- **Time awareness**: TF supports time-stamped transforms, allowing you to look up the state of the system at any
  historical or current time. This is particularly useful for synchronizing data from multiple sensors.
- **Distributed system**: TF can operate with a central server that contains all transform information, allowing all ROS
  components on any computer in the system to access the data.

<div class="text--center">
  <img src={tfTree} alt="Typical TF tree" />
</div>

The image above shows a typical TF tree in a simple robotic application. There are numerous coordinate frames with their
respective names, each with an arrow that points towards their reference frame. For instance, the transform of the
camera is defined in `world` frame, as is the `base` of the robot. The transform of the `object` is known with respect
to the camera and the pose of the `ur_tool0` frame is given by successive transformations along the robot links.

To drive the robot towards the object, one needs to know where the object is relative to the robot coordinate system.
Doing the math to chain all these transformations manually is cumbersome and prone to errors. TF provides an easy way
out by allowing to look up the transform between two frames that are part of the same tree directly:

```
header:
  stamp:
    sec: 1750404981
    nanosec: 370141910
  frame_id: base
child_frame_id: object
transform:
  translation:
    x: 0.250897
    y: 0.154403
    z: 0.097337
  rotation:
    x: 0.0
    y: -0.707107
    z: 0.707107
    w: 0.0
```

A transform message is stamped, i.e. it has a time associated with the transform. It also has a `frame_id` that refers
to the reference frame of the transform, and a `child_frame_id`, which is the name of the transform. Last but not least
are the values for the translation vector and the unit quaternion of the rotation.

Understanding the concept of TF, coordinate frames and their reference frames is crucial because if forms the foundation
for how spatial relationships are represented and managed in ROS-based robotic systems.

:::info

Read more about TF in
[the official ROS 2 documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html).

:::
