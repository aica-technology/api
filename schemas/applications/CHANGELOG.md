# CHANGELOG

Release Versions:

- [1-4-2](#163)
- [1-4-1](#141)
- [1-4-0](#140)
- [1-3-0](#130)
- [1-2-0](#120)
- [1-1-2](#112)
- [1-1-1](#111)
- [1-1-0](#110)
- [1-0-0](#100)

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