---
sidebar_position: 1
title: Inverse Kinematics
---

# Inverse Kinematics (IK) controllers

IK controllers operate in **task space**, allowing users to specify desired end-effector poses or motions, while the
controller computes the corresponding joint commands internally. This enables intuitive Cartesian control and
coordinated multi-joint motion without requiring direct manipulation of individual joints.

Depending on how the desired motion is expressed, IK controllers can be broadly categorized into **position-based**
and **velocity-based** controllers.

## Inverse kinematics position controller

### Overview

An **IK Position Controller** is designed to drive a robot toward a desired **Cartesian pose** (position and
orientation) of a controlled frame. Instead of commanding joint positions directly, the user specifies a target pose in
task space, and the controller computes the joint configuration that best realizes this pose.

The controller continuously evaluates the Cartesian pose error and updates the joint commands accordingly, typically
through an iterative process. This makes IK position control suitable for goal-driven tasks where the objective is to
**reach and maintain** a specific end-effector pose.

### Key characteristics and advantages

Compared to joint-space position controllers, an IK position controller offers several advantages:

- **Direct Cartesian goal specification:**
  Targets are expressed as end-effector poses (e.g., "move the tool to this pose"), which is often more intuitive than
  specifying joint angles.
- **Automatic joint coordination:**
  All joints are adjusted together to achieve the desired pose, ensuring consistent and coordinated motion.
- **Reactive behavior:**
  The controller continuously recomputes joint commands, allowing it to respond to disturbances or changes in the
  target pose.

In reactive or high-rate scenarios, IK position control can be harder to apply reliably, as valid solutions may not
always exist due to singularities or constraints. See more information in
[Position vs velocity IK control](/docs/product/Concepts/building-blocks/controllers/ik#position-vs-velocity-ik-control).

### Typical use cases

IK position control is well suited for:

- Reaching or holding a Cartesian pose
- Pick-and-place tasks
- Pose-based alignment or docking
- Interactive goal specification in Cartesian space

## Inverse kinematics velocity controller

### Overview

An **IK Velocity Controller** is designed to control a robot's motion by commanding
**joint velocities** such that a desired **task-space velocity** is achieved. Instead of following a predefined
time-parameterized trajectory, the controller continuously computes how fast each joint should move
*at the current control step* so that the end effector (or any controlled frame) moves with a specified linear and/or
angular velocity in Cartesian space.

This approach is fundamentally **reactive and continuous**, making it well suited for online control, teleoperation, and
interaction-driven tasks.

### Key characteristics and advantages

Compared to joint-level velocity or position controllers, an IK velocity controller offers several key advantages:

- **Task-space control:**
  Motion is specified directly in Cartesian space (e.g., "move the end effector forward at 5 cm/s"), which is often more
  intuitive and meaningful than joint commands.
- **Continuous motion generation:**
  Commands are generated at each control cycle, allowing smooth adaptation to changes in goals, sensor feedback, or
  external disturbances.
- **High responsiveness:**
  Since there is no fixed trajectory to follow, the controller can immediately react to new commands or constraints.
- **Coordinated joint motion:**
  Joint velocities are computed jointly to satisfy the task-space objective, ensuring coherent motion across all joints
  in the kinematic chain.

### Typical use cases

IK velocity control is particularly well suited for:

- Cartesian and visual servoing
- Teleoperation and joystick-based control
- Interaction and compliance tasks
- Online motion correction or refinement

## Position vs velocity IK control

In summary:

- **IK position controllers** are goal-oriented: they answer *"Where should the end effector be?"*
- **IK velocity controllers** are motion-oriented: they answer *"How should the end effector move right now?"*

Both approaches provide intuitive Cartesian control, but they are suited to different classes of applications depending
on whether the task is pose-driven or continuously evolving.

In practice, **IK position controllers are generally harder to use in reactive and real-time scenarios**. Computing a
valid inverse kinematics solution for a full Cartesian pose can fail due to singularities, joint limits, or conflicting
constraints, especially when targets change frequently or continuously. This makes position-based IK less robust for
high-rate, online control.

For this reason, **IK velocity control is often preferred in reactive applications**. A common pattern is to combine IK
velocity controllers with motion generators that produce desired Cartesian twists (see our
[Motion generation](/docs/product/concepts/robotics-concepts/motion-generation) page). This allows pose-driven behavior
to be expressed as smooth, continuous velocity commands, while retaining the robustness, responsiveness, and stability
properties of velocity-based IK control.

## Using it in AICA Core

Both an IK position and velocity controller are included in AICA Core by default. They can be used via a
**Cartesian pose** and **Cartesian twist** signal, respectively, that indicates the desired state. Additional parameters
can be modified to limit the Cartesian position or velocity.