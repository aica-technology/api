---
sidebar_position: 2
---

# Events

Events are discrete actions that change the dynamic state of an AICA application. They are handled and executed by the
Event Engine. For this reason, they are sometimes referred to as "state events".

## Event types

The following events can be used to dynamically update the state of a running application:

- Load or unload a component, controller or hardware interface
- Trigger a lifecycle transition (for example, configure, activate or deactivate) on a lifecycle component
- Start or stop a controller
- Set the value of a parameter on a loaded component or controller
- Call a service on a component

## Triggering events

Events can be triggered externally by a user through interactions with AICA Studio or via the REST API.
They can also be triggered internally by the Event Engine according to the application description as a result
of **predicates**, **transitions**, **sequences** or **conditions**.

### Predicates

Predicates are logical statements that evaluate to true or false and are used to indicate key states for either a
component or controller. AICA components and controllers broadcast any changes to their predicates to a global channel
in a message containing the source name, the predicate name, and the current value (true or false) of the predicate.

While the term "predicate" has several formal definitions in grammar, logic and mathematics, at AICA the [grammatical
definition](https://en.wikipedia.org/wiki/Predicate_(grammar)) is used when naming predicates.

Components define predicates depending on their function. A component that calculates if a given input state is within
some parameterized limits might have a predicate "is in bounds".

This definition of predicates allows application descriptions to trigger events in a very declarative way. For example:
> When component A _is in bounds_, load component B

Here, `component A` is the source of the predicate, `is in bounds` is the predicate itself, and `load component B` is
the event that is triggered when the predicate is true.

:::note

Events are only triggered on the [_rising edge_](https://en.wikipedia.org/wiki/Signal_edge) of a predicate value.

:::

### Transitions

The Event Engine allows components, controllers and hardware interfaces to be loaded and managed dynamically at runtime.
Often, it is desirable for the state change in one component to trigger the state change of another. This can apply in
the case of setting up an application, where a controller might need to be loaded only after the corresponding hardware
interface is loaded. It often also applies to the case of handling errors, such that when a component is unloaded or
enters into an error state, a controller or other component should be deactivated accordingly.

Every valid state transition on any component, controller or hardware interface in an AICA application can be easily
associated with one or more events to be triggered when that state transition occurs.

### Sequences

A sequence is a list of steps that are handled sequentially in order. Sequence steps can trigger any state event, while
intermediate sequence steps can also be configured with a time delay or to wait for a specific condition to resolve in
the application.

Sequences are a powerful tool to manage events in predetermined ways within the dynamic context of robotic applications.

### Conditions

Application states, including the current states of components, controllers, hardware interfaces or their predicates,
can be manipulated with logical operators in conditional statements to create more advanced event triggers.

> When component B _is **not** in bounds_, do ...

> When component A _is active_ **and** component B _is **not** in bounds_, do ...

AICA conditions support the "not", "all", "any", and "one of" operators (also known as the NOT, AND, OR and XOR
operators, respectively). If a condition does not trigger an event directly, it can still be used as the input to
another condition or to a conditional sequence step.

:::note

Events are only triggered on the rising edge of a conditional value.

:::