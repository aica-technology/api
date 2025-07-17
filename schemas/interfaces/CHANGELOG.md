# CHANGELOG

Release Versions:

- [1-0-0](#1-0-0)

## Upcoming changes

The 2-0-0 version of the interfaces schema includes major changes for more consistent and versatile usage.

### Breaking changes

- Forbid additional properties for signals, services and predicates
- Remove the `<interface>_name` property for all interface types
- Remove the `payload_format` property from services
- Expect default parameter values to be represented as real values according to the parameter type, not as string
  representations
- Forbid `parameter_state_type` when `parameter_type` is not `state`
- Forbid `custom_signal_type` when `signal_type` is not `other`

### Features

- Allow any real value default parameter value can now be any real value type, not just null or string
- More explicit validation rules for optional properties
- Include `integer` in parameter value type for languages that differentiate between integer and floating point numbers
- Refactor references and definitions to avoid circular or complex refs after bundling the schema
- Add `service_type`, `payload_description` and `payload_schema` properties with conditional validation rules to better
  describe service interfaces
- Add signal collection subschema

## 1-0-0

Initial release of the common interfaces schema defining parameters, predicates, services and signals.