---
sidebar_position: 2
title: Colliders
---

import boxCollider from './assets/box-collider.png'
import boxColliderExample from './assets/box-collider-example.gif'
import planeColliderExample from './assets/plane-collider-example.gif'
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

Each components' name refers to the geometric object that is used to check for collision:

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

Additionally, each collider variant may have different parameters that are required to define specific properties. For
the following colliders, the parameters define the size and shape of the geometric object.

- Box Collider: the side lengths of the box in the x, y, and z direction
- Cylinder Collider: the radius of the cylinder in the x-y plane and the height in the z direction
- Sphere Collider: the radius of the sphere

For the Plane Collider, the parameter "Flip normal" determines which side of the plane is considered "in collision"; by
default, this is when the z position of the target pose is negative in the coordinate system of the center pose.

:::note

Until collider visualizations are natively integrated in AICA Studio, these examples use RViz to visualize and move the
collision targets in space. RViz can be started directly from AICA Launcher.

<!-- TODO: Link to launcher page -->

:::

## Box Collider example

This example uses two Interactive Marker components for the target and center pose of the collider. The `Box Collider`
is parametrized to define a box of dimensions 10 by 20 by 5 centimeters around the center pose. Note that the
`Publish geometric object` flag is also toggled on to generate a visualization in RViz.

<div class="text--center">
  <img src={boxCollider} alt="Box Collider" />
</div>

Start the application from AICA Studio, then go to RViz. Add the interactive marker frame and the geometric object
marker as shown below. Observe how moving the interactive marker triggers the predicates of the collider component to
update when the target pose enters and exits the box.

<div class="text--center">
  <img src={boxColliderExample} alt="Box Collider example" />
</div>

<details>
  <summary>Application YAML</summary>

    ```yaml
    schema: 2-0-4
    dependencies:
      core: v4.4.1
    on_start:
      load:
        - component: box_collider
        - component: interactive_marker
        - component: interactive_marker_copy
    components:
      box_collider:
        component: aica_core_components::utility::BoxCollider
        display_name: Box Collider
        events:
          transitions:
            on_load:
              lifecycle:
                component: box_collider
                transition: configure
            on_configure:
              lifecycle:
                component: box_collider
                transition: activate
        parameters:
          publish_marker: true
          y_size: 0.2
          z_size: 0.05
        inputs:
          target: /interactive_marker/pose
          center: /interactive_marker_copy/pose
      interactive_marker:
        component: aica_core_components::ros::InteractiveMarker
        display_name: Target Interactive Marker
        parameters:
          frame: target
          initial_pose:
            - !!float 0.0
            - !!float 1.0
            - !!float 0.0
            - !!float 1.0
            - !!float 0.0
            - !!float 0.0
            - !!float 0.0
        outputs:
          pose: /interactive_marker/pose
      interactive_marker_copy:
        component: aica_core_components::ros::InteractiveMarker
        display_name: Center Interactive Marker
        parameters:
          frame: center
          initial_pose:
            - !!float 0.0
            - !!float 0.0
            - !!float 0.0
            - !!float 1.0
            - !!float 0.0
            - !!float 0.0
            - !!float 0.0
          handle_scale: 0.01
        outputs:
          pose: /interactive_marker_copy/pose
    graph:
      positions:
        components:
          box_collider:
            x: 660
            y: 60
          interactive_marker:
            x: 160
            y: 180
          interactive_marker_copy:
            x: 160
            y: 440
      edges:
        on_start_on_start_box_collider_box_collider:
          path:
            - x: 360
              y: 40
            - x: 360
              y: 120
        on_start_on_start_interactive_marker_interactive_marker:
          path:
            - x: 140
              y: 40
            - x: 140
              y: 240
        on_start_on_start_interactive_marker_copy_interactive_marker_copy:
          path:
            - x: 140
              y: 40
            - x: 140
              y: 500
        interactive_marker_copy_pose_box_collider_center:
          path:
            - x: 600
              y: 660
            - x: 600
              y: 440
    ```

</details>

## Plane Collider example

Switching from the Box Collider to the Cylinder or Sphere Collider components is straightforward. However, it is worth
looking at the Plane Collider separately because it is not immediately obvious on which side of the plane the target _is_
_in collision_ with the plane and on which side it _is not in collision_.

As per the component description, a target pose is considered _in collision_ if it has a negative z coordinate with
respect to the center pose. That means, if a table is defined by a Plane Collider and the z axis of the center pose,
which corresponds to the normal to the table, points up, the target pose is in collision as soon as it reached (or
penetrated) the table.

:::tip

This behavior can be inverted with the "Flip normal" parameter. The direction of the z axis will be flipped internally.

:::

The example below shows an application that is stopped entirely if the robot end-effector collides with the plane. This
demonstrates how soft safety mechanisms can be implemented in AICA Studio. As soon as the target pose, which is the
end-effector in this case, has a negative z coordinate relative to the center pose, the _in collision_ predicate fires
and the application is stopped immediately.

<div class="text--center">
  <img src={planeColliderExample} alt="Plane Collider example" />
</div>

<details>
  <summary>Application YAML</summary>

    ```yaml
      schema: 2-0-4
      dependencies:
        core: v4.4.1
      on_start:
        load:
          - component: interactive_marker
          - component: interactive_marker_copy
          - component: plane_collider
          - hardware: hardware
      components:
        interactive_marker:
          component: aica_core_components::ros::InteractiveMarker
          display_name: Target Interactive Marker
          parameters:
            frame: target
            initial_frame: tool0
          outputs:
            pose: /interactive_marker/pose
        interactive_marker_copy:
          component: aica_core_components::ros::InteractiveMarker
          display_name: Center Interactive Marker
          parameters:
            frame: center
            initial_pose:
              - 0.1
              - !!float 0.0
              - 0.3
              - !!float 1.0
              - !!float 0.0
              - !!float 0.0
              - !!float 0.0
            handle_scale: 0.01
          outputs:
            pose: /interactive_marker_copy/pose
        plane_collider:
          component: aica_core_components::utility::PlaneCollider
          display_name: Plane Collider
          events:
            predicates:
              is_in_bounds:
                application: stop
            transitions:
              on_load:
                lifecycle:
                  component: plane_collider
                  transition: configure
              on_configure:
                lifecycle:
                  component: plane_collider
                  transition: activate
          parameters:
            publish_marker: true
          inputs:
            target: /hardware/robot_state_broadcaster/cartesian_state
            center: /interactive_marker_copy/pose
      hardware:
        hardware:
          display_name: Hardware Interface
          urdf: Generic six-axis robot arm
          rate: 100
          events:
            transitions:
              on_load:
                load:
                  - controller: robot_state_broadcaster
                    hardware: hardware
                  - controller: ik_position_controller
                    hardware: hardware
          controllers:
            robot_state_broadcaster:
              plugin: aica_core_controllers/RobotStateBroadcaster
              outputs:
                cartesian_state: /hardware/robot_state_broadcaster/cartesian_state
              events:
                transitions:
                  on_load:
                    switch_controllers:
                      hardware: hardware
                      activate: robot_state_broadcaster
            ik_position_controller:
              plugin: aica_core_controllers/position/IKPositionController
              inputs:
                command: /interactive_marker/pose
              events:
                transitions:
                  on_load:
                    switch_controllers:
                      hardware: hardware
                      activate: ik_position_controller
      graph:
        positions:
          components:
            interactive_marker:
              x: 160
              y: 180
            interactive_marker_copy:
              x: 160
              y: 440
            plane_collider:
              x: 680
              y: 500
          hardware:
            hardware:
              x: 1220
              y: -20
        edges:
          on_start_on_start_interactive_marker_interactive_marker:
            path:
              - x: 140
                y: 40
              - x: 140
                y: 240
          on_start_on_start_interactive_marker_copy_interactive_marker_copy:
            path:
              - x: 140
                y: 40
              - x: 140
                y: 500
          on_start_on_start_plane_collider_plane_collider:
            path:
              - x: 600
                y: 40
              - x: 600
                y: 560
          interactive_marker_copy_pose_plane_collider_center:
            path:
              - x: 580
                y: 660
              - x: 580
                y: 880
          plane_collider_is_in_bounds_on_stop_on_stop:
            path:
              - x: 1100
                y: 720
              - x: 1100
                y: 1000
              - x: -20
                y: 1000
              - x: -20
                y: 140
          hardware_hardware_robot_state_broadcaster_cartesian_state_plane_collider_target:
            path:
              - x: 620
                y: 520
              - x: 620
                y: 840
          interactive_marker_pose_hardware_hardware_ik_position_controller_command:
            path:
              - x: 1140
                y: 400
              - x: 1140
                y: 780
    ```

</details>
