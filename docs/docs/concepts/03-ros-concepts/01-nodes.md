---
sidebar_position: 1
---

# Nodes

Nodes are at the core of ROS. They are independent processes responsible for performing some kind of computation.

Most importantly, nodes expose discoverable interfaces that allow them to be interconnected and managed in a network
graph.

Nodes can publish messages and subscribe to messages from other nodes on communication channels known as topics, and
have parameters to configure their behavior. Nodes can also act as service and action servers for other client nodes to
make requests and get a response message or trigger specific actions.

By designing nodes as pure functional objects that perform a specialized task with determined inputs, outputs and
parameters, they become more modular and re-usable in complex applications. However, care should be taken to use
standard messages and behaviors when implementing node interfaces; if any two nodes are expected to be connected in a
network graph, they must have compatible interfaces.

:::info

Read more about nodes
in [the official ROS 2 documentation](http://docs.ros.org/en/iron/Concepts/Basic/About-Nodes.html).

:::