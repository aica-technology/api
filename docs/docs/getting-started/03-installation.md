---
sidebar_position: 3
title: Installation
---

## Pre-requisites

AICA software is distributed with Docker images and executed with Docker containers.

Before proceeding, be sure to [install Docker Engine](https://docs.docker.com/engine/install/) on the host machine.

## Logging in to the AICA package registry

AICA software packages are hosted in a private container registry. Use a valid [license file](./02-licensing.md) to
authenticate docker to login and pull images from the registry using the following command:

```shell
cat aica-license.toml | docker login registry.licensing.aica.tech -u USERNAME --password-stdin
```

:::note

The `USERNAME` in the command can be left as-is and does not have to be replaced with any user-specific information.
This is because the authentication layer ignores the username and only uses the license key supplied as the password.

:::

## Configuring AICA packages with a manifest file

As seen in [Concepts: Introduction](../concepts/01-intro.md), AICA software comprises a collection of multiple packages
around a unified framework. The base package includes the Dynamic State Engine, a core collection of components and
controllers, the REST API and Developer Interface UI. Add-on packages include additional components or hardware
collections.

A runtime application image is configured using a simple **manifest file** defining the version of the base package
to use and optionally defining additional add-on packages. The manifest file contains a custom docker syntax header
pointing to AICA's app-builder tool, and the `docker build` command is used to bundle all listed packages into a final
runtime image.

### Configuring a minimal runtime image with the core base package

The manifest file must contain a syntax header and a list of packages. The minimal version of the manifest includes
only the base package. The version of the base package can be changed according to the latest release.

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/app-builder:v1

[packages]
"@aica/base" = "v2.1.0"
```

### Configuring a runtime image with add-on packages

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/app-builder:v1

[packages]
"@aica/base" = "v2.1.0"

# add components
"@aica/components/signal-processing" = "v1.0.1"

# add hardware collections 
"@aica/collections/ur-collection" = "v3.0.4"
```

An AICA license includes specific entitlements that determine which add-on packages and versions can be accessed and
used.
AICA support will provide a package manifest with the latest included add-ons together with the respective license key.

For a full list of available add-ons for components and hardware collections, along with their latest versions and
release notes, contact your AICA representative.

## Building an AICA runtime application image

For a manifest file saved as `aica-package.toml`, use the following command to build the runtime application.

```shell
docker build -f aica-package.toml -t aica-runtime .
```

The command `docker image ls | grep aica-runtime` should then list the `aica-runtime` image.

Continue to the next section to learn how to start the application container.
