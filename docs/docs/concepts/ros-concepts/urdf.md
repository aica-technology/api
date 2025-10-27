---
sidebar_position: 6
title: URDF
---

import sixAxisArm from './assets/six-axis-arm.png'

# Unified Robot Description Format

In ROS, a **URDF** is an XML-based format used to describe the physical configuration of a robot. The terms URDF and
**robot description** are often used interchangeably. URDF files define the robot's structure in terms of:

- **Links**: Rigid bodies representing parts of the robot (e.g. arms, base, sensors).
- **Joints**: Connections between links, specifying how they move relative to each other (e.g. revolute, prismatic,
  etc.).
- **Visuals and Collisions**: 3D geometry for visualization and collision checking.
- **Inertial Properties**: Mass and inertia of links for physics simulation.

URDFs are essential for simulation, visualization, motion planning, and interfacing with tools like
[ros2_control](./controlling-robots). They provide a standardized way to represent a robot's kinematic and dynamic
properties.

:::tip

Different tools or simulators might require different formats for a robot description (e.g. SDF for Gazebo, USD and XRDF
for Isaac Sim, MJCF for MuJoCo, etc.). They all share the same basic ideas and there exists a wide range of open source
conversion tools.

:::

The XML file belows shows the URDF of a very simple six axis robot arm that consists of a series of links connected by
revolute joints. Each definition of a link is followed by a joint that connects it to its child link. Links have an
optional `visual` tag that says how it should be visualized. In this case, cylinders as geometrical primitive are used
to give the robot a certain look. URDFs of real hardware usually refer to 3D model files (STL, DAE) for this. Note that
the links are directly related to [TF](./tf) as every link will become its own coordinate frame. The image below shows
the visualization of this robot description with the links as frames in TF.

```xml
<?xml version="1.0" ?>
<robot name="six_axis_arm_robot">
  <material name="generic/Yellow">
    <color rgba="1 1 0 1"/>
  </material>
  <material name="generic/LightGrey">
    <color rgba="0.6 0.6 0.6 1"/>
  </material>
  <link name="world"/>
  <joint name="to_world" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="base_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.01"/>
      <geometry>
        <cylinder length="0.02" radius="0.08"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
  </link>
  <joint name="joint_0" type="revolute">
    <parent link="base_link"/>
    <child link="link_0"/>
    <origin rpy="0 0 0" xyz="0 0 0.085"/>
    <axis xyz="0 0 1"/>
    <limit effort="30.0" lower="-3.14" upper="3.14" velocity="5.0"/>
  </joint>
  <link name="link_0">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.13" radius="0.05"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="link_0"/>
    <child link="link_1"/>
    <origin rpy="0 0 0" xyz="0 0.0825 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="30.0" lower="-1.57" upper="1.57" velocity="5.0"/>
  </joint>
  <link name="link_1">
    <visual>
      <origin rpy="1.57 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.07" radius="0.03"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.21"/>
      <geometry>
        <cylinder length="0.42" radius="0.03"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
  </link>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin rpy="1.57 0 3.14" xyz="0 -0.03375 0.39"/>
    <axis xyz="0 0 1"/>
    <limit effort="30.0" lower="-1.57" upper="1.57" velocity="5.0"/>
  </joint>
  <link name="link_2">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.07" radius="0.0225"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
    <visual>
      <origin rpy="0 1.57 0" xyz="-0.1525 0 -0.052500000000000005"/>
      <geometry>
        <cylinder length="0.35" radius="0.0225"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
    <visual>
      <origin rpy="0 0 0" xyz="-0.306 0 -0.023333333333333338"/>
      <geometry>
        <cylinder length="0.07" radius="0.02"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
  </link>
  <joint name="joint_3" type="revolute">
    <parent link="link_2"/>
    <child link="link_3"/>
    <origin rpy="0 1.57 0" xyz="-0.31499999999999995 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="30.0" lower="-1.57" upper="1.57" velocity="5.0"/>
  </joint>
  <link name="link_3">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 -0.01125"/>
      <geometry>
        <cylinder length="0.07" radius="0.0225"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
  </link>
  <joint name="joint_4" type="revolute">
    <parent link="link_3"/>
    <child link="link_4"/>
    <origin rpy="1.57 0 0" xyz="0 0 -0.0575"/>
    <axis xyz="0 1 0"/>
    <limit effort="30.0" lower="-1.57" upper="1.57" velocity="5.0"/>
  </joint>
  <link name="link_4">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.07" radius="0.0225"/>
      </geometry>
      <material name="generic/LightGrey"/>
    </visual>
  </link>
  <joint name="joint_5" type="revolute">
    <parent link="link_4"/>
    <child link="link_5"/>
    <origin rpy="0 0 3.14" xyz="0 0 0.045000000000000005"/>
    <axis xyz="0 0 1"/>
    <limit effort="30.0" lower="-1.57" upper="1.57" velocity="5.0"/>
  </joint>
  <link name="link_5">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.02" radius="0.0225"/>
      </geometry>
      <material name="generic/Yellow"/>
    </visual>
  </link>
  <joint name="arm-tool0" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="link_5"/>
    <child link="tool0"/>
  </joint>
  <link name="tool0"/>
</robot>
```

<div class="text--center">
  <img src={sixAxisArm} alt="Visualization of simple URDF" />
</div>

:::info

Check the detailed XML specifications of URDF files in [the official documentation](https://wiki.ros.org/urdf/XML).

:::
