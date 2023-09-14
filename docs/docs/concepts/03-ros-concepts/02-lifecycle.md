---
sidebar_position: 2
---

# Lifecycle nodes

Lifecycle nodes, also referred to as "managed" nodes, extend the common interfaces of regular nodes with an internal
state machine following the concept of _lifecycle states_.

Nodes are normally launched as a collection of processes based on a launch configuration file. In complex applications,
it may not be desirable for all node processes to start executing right away. For example, a computer vision processing
node should not perform any calculations until a camera sensor node is ready.

Lifecycle nodes present a solution to this problem by distinguishing between different activation states. Developers can
then implement different node behaviors depending on the current internal state.

A lifecycle node has four primary states:

- Unconfigured
- Inactive
- Active
- Finalized

There are seven primary transitions between the states:

- Create
- Configure
- Activate
- Deactivate
- Cleanup
- Shutdown
- Destroy

A lifecycle node is a managed node because the transitions between primary lifecycle states are normally triggered by an
external process. In other words, a lifecycle node in the inactive state stays inactive until an external process (for
example, another node) requests a transition (for example, from inactive to active using the `activate` transition).

Another key benefit of lifecycle nodes is the ability to build sophisticated error handling and recovery behavior. An
error can be handled differently depending on the current state or transition. Instead of the node crashing and shutting
down completely, a properly handled error can recover the lifecycle node to an unconfigured state.

:::info

Read more about managed nodes in [the ROS 2 design document](https://design.ros2.org/articles/node_lifecycle.html).

:::

Using lifecycle nodes is one way to dynamically manage the behavior of an application at runtime. Another approach is
to use dynamic composition, which is described on the next page.
