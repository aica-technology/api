---
sidebar_position: 6
title: Migrating from older versions of `package-builder`
---

# Migrating from older versions of `package-builder`

While we try to keep the syntax of `package-builder` as stable as possible, sometimes we need to make breaking changes to improve the developer experience or to fix bugs. This document will help you migrate your existing `package-builder` configuration to the latest version.

## Migrating from `0.0.x`

Here are the major changes when moving from `0.0.x` to `1.0.0`:

- `[metadata.version]` is now mandatory and the previous fields `[metadata.name]` and `[metadata.ros-name]` are now respectively `[metadata.collection.name]` and `[metadata.collection.ros-name]`.
- `[build.environment.aica.libraries]` and `[build.environment.aica.ros]` have been merged into `[build.dependencies]`.
- `[build.cmake_args]` has been renamed `[build.cmake-args]`.
- `[build.environment.ssh]` and `[build.environment.image]` have been moved to `[build.ssh]` and `[build.image]` respectively.

:::note
`package-builder` will now do extra checks to ensure all your `[build.dependencies]` are compatible with `[build.image]`. In order to do that, you will need to upgrade the version in those fields to newer ones which incorporate the necessary metadata.
:::
