# CHANGELOG

Release Versions:

- [2-1-0](#2-1-0)
- [2-0-6](#2-0-6)
- [2-0-5](#2-0-5)
- [2-0-4](#2-0-4)
- [2-0-3](#2-0-3)
- [2-0-2](#2-0-2)
- [2-0-1](#2-0-1)
- [2-0-0](#2-0-0)
- [1-4-2](#1-4-2)
- [1-4-1](#1-4-1)
- [1-4-0](#1-4-0)
- [1-3-0](#1-3-0)
- [1-2-0](#1-2-0)
- [1-1-2](#1-1-2)
- [1-1-1](#1-1-1)
- [1-1-0](#1-1-0)
- [1-0-0](#1-0-0)

## 2-1-0

This update adds variables and assignments to the application schema, which allow parameter values to be set and reused
from multiple sources. The definition of the hardware interface now supports an explicit reference to database entries.

From this version onwards, referring to hardware interfaces by name in the `urdf` property should be considered
deprecated; a local database ID or full XML content should be used instead.

- feat: add support for variables and assignments (#386)
- feat: define database entry for referencing hardware (#405)
- fix: use 2-0-1 version of interfaces schema (#426)

## 2-0-6

This patch makes parameter values more type-safe by providing the option to explicitly specify the parameter type for
component and controller parameters or when setting a parameter through an event. The previous way of specifying the
parameter value directly is still compatible with this new version.

## 2-0-5

This patch allows parameter values to have the `null` type, which is intended to represent an unset or "empty"
parameter value. This should be seen as distinct from an empty string value `""` or any other falsey value.

## 2-0-4

This patch relaxes the pattern properties for parameter and frame names in the application schema. Names are now allowed
to start with any alphanumeric character and can contain underscores, full stops, and dashes.

### Changelog

- feat: more permissive names for frames and parameters (#260)

## 2-0-3

This patch adds new optional properties to components, hardware and the top-level application.

### Changelog

- feat: add intra process comms field to component schema (#192)
- feat: add joint positions to application schema (#215)
- feat: add custom tf_prefix and reference_frame support in schema (#216)

## 2-0-2

This patch includes application events under the graph trigger button events, giving users the option to stop the
application from a trigger button instead of the main application controls in Studio.

## 2-0-1

This patch replaces the version constraint regex for the package dependencies with a simpler and more permissive case.
The previously used regex lookaround expressions were not supported in all browsers, including Safari.

## 2-0-0

### New features in 2-0-0

- Application dependencies can be defined under the top-level `dependencies` property, including the core image version
  and additional packages
- Application metadata can be defined under the top-level `metadata` property including a name, description and tags
- Any additional user data can be included in an application under the top-level `userdata` property
- Interactive UI buttons and all position data is now defined under a top-level `graph` property
- A running application can be stopped with the `application: stop` event from any event source
- Components, controllers and hardware support dedicated transition events such as `on_load`, `on_activate`
  and `on_error` which behave as event triggers similar to predicates
- Lifecycle components have access to additional error handling with the `on_configure_failure`, `on_activate_failure`,
  `on_error` and `on_error_recovery` transition events
- Condition sources for sequence steps and conditions now include component, controller, hardware or sequence states in
  addition to the previous component or controller predicate sources
- Hardware control rate can be supplemented with a `rate_tolerance` to determine the allowable deviation from the
  intended control rate, and an optional `strict` flag that immediately shuts down the hardware in case of rate
  deviation or other error
- Static frames can be defined in the application with a position, orientation, name, and reference frame under the new
  top-level `frames` property
- Graph positions can be expressed for all elements, including sequences, conditions and a stop application node
- Edge path information for custom edge routing can be stored under the `edges` sub-property of `graph`
- Sequences have a property to support automatic looping
- Sequences, conditions and controllers now support display names
- The event structures for lifecycle transitions, service calls and setting parameters now always require the target
  component to be specified, removing the previous "shorthand" for self-targeting events

### Breaking changes

Refer to the migration guide in [README.md](./README.md#migrating-from-1-4-x-to-2-0-0) for more information.

## 1-4-2

Version 1-4-2 adds support for the `set` parameter event in buttons.

## 1-4-1

Version 1-4-1 adds support for controller predicates in conditions and sequences.

## 1-4-0

Version 1-4-0 adds support for predicate events on controllers.

## 1-3-0

Version 1-3-0 adds support for service calls on controllers.

## 1-2-0

Version 1-2-0 adds a new syntax to manage sequential events as an array of steps to be handled in order. Sequence steps
are either standard state events or conditional blocks; the latter are used either to wait for a condition, predicate
or fixed time interval, or to assert the current value of a condition or predicate.

## 1-1-2

Version 1-1-2 allows a single item in the list of transitions of lifecycle events to be just the transition keyword
instead of a full transition object.

## 1-1-1

Version 1-1-1 fixes a problem with the recursion inside the conditions schema.

### Changelog

- fix: update conditions schema (#114)

## 1-1-0

Version 1-1-0 fixes an error in the application schema related to required field in the hardware schema.

### Changelog

- fix: required fields for hardware schema (#97)

## 1-0-0

Version 1-0-0 marks the version for the first software release. From now on, all changes must be well documented and
semantic versioning must be maintained to reflect patch, minor or major changes.