# CHANGELOG

Release Versions:

- [1-0-1](#1-0-1)
- [1-0-0](#1-0-0)

## 1-0-1

This addition to the extension schema adds an optional property to hold the name and version of the package produced by
the [package-builder](https://ghcr.io/aica-technology/package-builder) tool. The name and version are conventionally
defined in an `aica-package.toml` file. The intention is not for the extension author to populate this field, but rather
for the core API to associate the extension with its source package for a particular AICA System configuration and
automatically fill in this field.

## 1-0-0

Initial release of the extensions schema for components and controllers.