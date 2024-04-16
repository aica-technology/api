# CHANGELOG

Release Versions:

- [1-3-0](#130)
- [1-2-0](#120)
- [1-1-0](#110)
- [1-0-0](#100)

## 1-3-0

As of version 1-2-0, controllers can also have the field `predicates` to list the predicates provided by the controller.

### Changelog

- feat(controller-schema): add predicates (#145)

## 1-2-0

As of version 1-2-0, controllers can also have the field `services` to list the services provided by the controller.

### Changelog

- feat(controller-schema): add services (#138)

## 1-1-0

Version 1-1-0 adds a new property `optional` to the parameter schema to indicate if the user needs to specify a value in
case a parameter doesn't have a default value.

### Changelog

- feat(parameter-schema): add new keyword optional for the parameter schema (#111)

## 1-0-0

Version 1-0-0 marks the version for the first software release. From now on, all changes must be well documented and
semantic versioning must be maintained to reflect patch, minor or major changes.