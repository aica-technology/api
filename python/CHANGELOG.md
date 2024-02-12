# CHANGELOG

Release Versions:

- [2.0.0](#200)
- [1.2.1](#121)
- [1.2.0](#120)
- [1.1.0](#102)
- [1.0.2](#102)
- [1.0.1](#101)
- [1.0.0](#100)

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