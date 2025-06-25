---
sidebar_position: 2
title: Colliders
---

import colliderInterfaces from './assets/collider-interfaces.png'

# Colliders

The core components in AICA Studio contain several so-called _Collider_ components that detect whether a specific target
pose is inside or outside a virtual geometric object defined around a center pose. This functionality is crucial for
many robotic applications because it provides the ability to:

- **Avoid collisions** by ensuring that robot parts or tools do not enter restricted or hazardous areas;
- **Enable safe interaction** by detecting when a gripper or end-effector is within a workspace or in contact with an
  object;
- **Trigger context-aware actions** such as stopping or slowing down movement, initiating grasping, or sending alerts
  when a target enters or exits a region.

The components' name refers to the geometric object that is used to check for collision:

- Box Collider
- Cylinder Collider
- Sphere Collider
- Plane Collider

## Interfaces

All colliders have the same input signals and predicates, as seen in the image below. The target pose input refers to
the Cartesian pose that is checked against the collider region. The center pose input defines the center of the region
(the barycenter of the geometric object). The predicate "is in collision" is true whenever the target is within the
collider region (or, in the case of the Plane Collider, when the target is below the plane), and conversely the
predicate "is not in collision" is true in the opposite case.

<div class="text--center">
  <img src={colliderInterfaces} alt="Collider interfaces" />
</div>

### Parameters

Additionally, each collider variant has different parameters that are required to define specific properties. For the
following colliders, the parameters define the size and shape of the geometric object.

- Box Collider: the side lengths of the box in the x, y, and z direction
- Cylinder Collider: the radius of the cylinder in the x-y plane and the height in the z direction
- Sphere Collider: the radius of the sphere

For the Plane Collider, the parameter "Flip normal" determines which side of the plane is considered "in collision"; by
default, this is when the z position of the target pose is negative in the coordinate system of the center pose.

:::tip

See [this page](../../../examples/core-components/colliders.md) for examples using collider components in AICA Studio.

:::