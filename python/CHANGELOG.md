# CHANGELOG

Release Versions:

- [3.1.1](#311)
- [3.1.0](#310)
- [3.0.0](#300)
- [2.1.0](#210)
- [2.0.0](#200)
- [1.2.1](#121)
- [1.2.0](#120)
- [1.1.0](#102)
- [1.0.2](#102)
- [1.0.1](#101)
- [1.0.0](#100)

## 3.1.1

Version 3.1.1 is a patch release to ensure that content sent with the `set_application` function is always converted to
JSON format.

### Fixes

- Convert loaded application to json (#349)

## 3.1.0

Version 3.1.0 of the AICA API client is compatible with AICA Core v4.3.0. This version introduces authentication and
access scopes to the API server. An API key with appropriate scopes is required to access the respective endpoints and
functionalities.

### Features

- Add support for the upcoming auth update (#207)

## 3.0.0

Version 3.0.0 of the AICA API client is compatible with the new AICA Core version 4.0. It supports additional methods to
query the Core, API server and protocol versions and the sub-versions of installed packages through license metadata.

This release removes the previously deprecated method `wait_for_predicate` which has been replaced by the specific
`wait_for_component_predicate` or `wait_for_controller_predicate` methods.

Similarly, it marks the `call_service` method as deprecated in this version and introduces the `call_component_service`
and `call_controller_service` methods as preferred alternatives.

This release also introduces two new methods to manage sequences: `manage_sequence` to start, restart or abort a named
sequence and `wait_for_sequence` to wait until a sequence is inactive, active or aborted.

## 2.1.0

- Support hardware and controller states and predicates in `wait_for` functions (#156)
- Define more detailed feature and function compatibility between versions (#158)

## 2.0.0

This version of the AICA API client is compatible with the new AICA API server version 3.0 by using Socket.IO instead of
raw websockets for run-time data. This change breaks backwards compatibility for API server versions v2.x and below.
It uses Socket.IO instead of raw websockets for run-time data required by the new AICA framework API version 3.0.

### Breaking changes

- refactor!: use Socket.IO client instead of websockets for run-time data (#95)

## 1.2.1

- Correct typehints for setting parameters (#96)

## 1.2.0

- Parse YAML file in set_application if it exists on client machine (#90)

## 1.1.0

- Fix JSON format for setting parameter value (#80)
- Add function to set the lifecycle transition on a component (#81)

## 1.0.2

Patch the endpoint URL to correctly address API version 2.0

## 1.0.1

Version 1.0.1 fixes a relative import issue.

## 1.0.0

Version 1.0.0 marks the version for the first software release. From now on, all changes must be well documented and
semantic versioning must be maintained to reflect patch, minor or major changes.
