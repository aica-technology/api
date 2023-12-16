---
sidebar_position: 2
---

# Events

Events are discrete actions that change the dynamic state of an AICA application. They are handled and executed by the
Dynamic State Engine. For this reason, they are sometimes referred to as "state events".

## Event types

The following events can be used to dynamically update the state of a running application:

- Load or unload a component, controller or hardware interface
- Trigger a lifecycle transition (for example, configure, activate or deactivate) on a lifecycle component
- Start or stop a controller
- Set the value of a parameter on a loaded component or controller
- Call a service on a component

## Triggering events

Events can be triggered externally by a user through interactions with the Developer Interface UI or via the REST API.
They can also be triggered internally by the Dynamic State Engine according to the application description as a result
of component **predicates** and **conditions**.

### Predicates

Predicates are logical statements about a component that evaluate to true or false and are used to indicate key
component states. AICA components broadcast their predicates to a global channel in a predicate message containing the
component name, the predicate name, and the current value (true or false) of the predicate.

While the term "predicate" has several formal definitions in grammar, logic and mathematics, at AICA the [grammatical
definition](https://en.wikipedia.org/wiki/Predicate_(grammar)) is used when naming predicates.

Components define predicates depending on their function. A component that calculates if a given input state is within
some parameterized limits might have a predicate "is in bounds".

This definition of predicates allows application descriptions to trigger events in a very declarative way. For example:
> When component A _is in bounds_, load component B

Here, `component A` is the source of the predicate, `is in bounds` is the predicate itself, and `load component B` is the
event that is triggered when the predicate is true.

:::note

Events are only triggered on the [_rising edge_](https://en.wikipedia.org/wiki/Signal_edge) of a predicate value.

:::

### Conditions

Predicate statements can be manipulated with logical operators in conditional statements to create more advanced event
triggers.

> When component B _is **not** in bounds_, do ...

> When component A _is active_ **and** component B _is **not** in bounds_, do ...

AICA conditions support the NOT, AND, OR and XOR operators (also known as the "not", "all", "any", and "one of"
operators, respectively).

:::note

Events are only triggered on the rising edge of a conditional value.

:::