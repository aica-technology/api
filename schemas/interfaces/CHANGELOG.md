# CHANGELOG

Release Versions:

- [2-0-1](#2-0-1)
- [2-0-0](#2-0-0)
- [1-0-0](#1-0-0)

## 2-0-1

Version 2-0-1 re-releases the 2-0-0 version that was previously overwritten by an incorrect CI workflow.

## 2-0-0

The 2-0-0 version of the interfaces schema includes major changes for more consistent and versatile usage.

### Breaking changes

- Forbid additional properties for signals, services and predicates
- Remove the `payload_format` property from services
- Remove the `signal_types` and `reconfigurable_type` properties from signals
- Remove the `signal_type.schema.json` subschema and define inline in signal.schema.json
- Expect default parameter values to be represented as real values according to the parameter type, not as string
  representations
- Forbid `parameter_state_type` when `parameter_type` is not `state`
- Reduce permitted options in `signal_type` by removing all encoded state types except for `state` and replace `other`
  with `external`
- Remove the `default_value` annotation for default signal topics

### Features

- Allow any real value default parameter value, not just null or string
- Use `signal_state_type` to define the encoded state variant when `signal_type` is `state`
- More explicit validation rules for optional properties
- Include `integer` in parameter value type for languages that differentiate between integer and floating point numbers
- Refactor references and definitions to avoid circular or complex refs after bundling the schema
- Add `service_type`, `payload_description` and `payload_schema` properties with conditional validation rules to better
  describe service interfaces
- Add assignment subschema
- Add parameter validation options

## 1-0-0

Initial release of the common interfaces schema defining parameters, predicates, services and signals.