# CHANGELOG

Release Versions:

- [1-0-2](#1-0-2)
- [1-0-1](#1-0-1)
- [1-0-0](#1-0-0)

## Upcoming changes in draft 1-0-3

The bundled extension.types.schema.json patches the `payload_schema` object to have no required properties to
resolve an issue with API model marshalling.

## 1-0-2

Version 1-0-2 uses the patched 2-0-1 version of the interfaces schema.

## 1-0-1

This addition to the extension schema adds an optional property to hold the name and version of the package produced by
the [package-builder](https://ghcr.io/aica-technology/package-builder) tool. The name and version are conventionally
defined in an `aica-package.toml` file. The intention is not for the extension author to populate this field, but rather
for the core API to associate the extension with its source package for a particular AICA System configuration and
automatically fill in this field.

## 1-0-0

Initial release of the extensions schema for components and controllers.