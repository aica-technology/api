# CHANGELOG

Release Versions:

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

## 2-0-0

### New features in 2-0-0

- Application dependencies can be defined under the top-level `dependencies` property, including the base image version
  and additional packages
- Application metadata can be defined under the top-level `metadata` property including a name, description and tags
- Any additional user data can be included in an application under the top-level `userdata` property
- A running application can be stopped with the `application: stop` event from any event source
- Components, controllers and hardware support dedicated transition events such as `on_load`, `on_activate`
  and `on_error` which behave as event triggers similar to predicates
- Condition sources for sequence steps and conditions now include component, controller, hardware or sequence states in
  addition to the previous component or controller predicate sources
- Hardware control rate can be supplemented with a `rate_tolerance` to determine the allowable deviation from the
  intended control rate, and an optional `strict` flag that immediately shuts down the hardware in case of rate
  deviation or other error
- Sequences support additional properties including display name, position on the graph and automatic looping
- Sequences and conditions support display names

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