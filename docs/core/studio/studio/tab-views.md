---
sidebar_position: 4
title: Tab views
---

# Live views

Monitor and visualize the state of a running application with the following live views.

## Scene

<!-- TODO: Settings, Add Component, Scene List and screenshots-->

## Code

<!-- TODO: brief explanation of YAML code and generate graph button, and new screenshot -->

## 3D view / Graph view

Depending on whether the [application graph](./graph.md) or the [3D view](./3d.md) is active for the main view, this tab
will show the other view. The right panel mode of the graph view is non-interactive for modifying nodes, edges or 
parameters but allows zooming, panning and pressing trigger buttons for configured events. When the 3D view is in the
right panel, the 3D view settings are accessed from a mini-panel rather than the Settings sub-tab of the Scene tab.

## Live Data

View the data running on signals and other background topics on the ROS 2 network.

Selecting a signal edge in the graph view while the application is running will automatically select the appropriate
signal topic for visualization in this tab.

<!-- TODO: new screenshot -->

## Logs

The logs view shows log entries from the running system color-coded by severity level.

<!-- TODO: new screenshot -->

## Live Table

The live state table gives a compact overview of which components and controllers are loaded, what state they are in,
and which predicates and conditions are true or false.

<!-- TODO: new screenshot -->
