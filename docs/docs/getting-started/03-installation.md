---
sidebar_position: 3
title: Installation
---

## Pre-requisites

AICA software is distributed with Docker images and executed with Docker containers.

Before proceeding, [install Docker Engine](https://docs.docker.com/engine/install/) on the host machine.

:::info

For Ubuntu users, make sure to follow
the [post installation steps](https://docs.docker.com/engine/install/linux-postinstall/) to create the `docker` group
and add your user.

:::

## The AICA package registry

As seen in [Concepts: Introduction](../concepts/01-intro.md), AICA software comprises a collection of multiple packages
around a unified framework. The base package includes the Dynamic State Engine, a core collection of components and
controllers, the REST API and Developer Interface UI. Add-on packages include additional components or hardware
collections.

AICA software packages are hosted as docker images in a private container registry. Viewing and downloading packages
from the registry requires a valid [license](./02-licensing.md).

### Listing available AICA packages

To list available packages and versions, go to the official AICA registry page
at [https://registry.licensing.aica.tech](https://registry.licensing.aica.tech) and enter your license key.

:::tip

An AICA license includes specific entitlements that determine which add-on packages and versions can be accessed and
used. To discover and access additional components and hardware collections, contact your AICA representative to upgrade
your license.

:::

### Logging in to the AICA package registry

To authenticate docker to login and pull images from the registry, run the following command:

```shell
cat aica-license.toml | docker login registry.licensing.aica.tech -u USERNAME --password-stdin
```

:::note

The `USERNAME` in the command can be left as-is and does not have to be replaced with any user-specific information.
This is because the authentication layer ignores the username and only uses the license key supplied as the password.

:::

## Configuring AICA packages with a manifest file

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
"@aica/base" = "v3.1.1"
```

### Configuring a runtime image with add-on packages

A manifest can include additional components and hardware collections as add-on packages. For any available package
[listed in the AICA registry](#listing-available-aica-packages), specify the package and version with the `@aica/`
prefix. The following example manifest file includes two add-on packages: version 1.0.1 of the
`components/signal-processing` component package and version 3.04 of the `collections/ur-collection` hardware collection
package.

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/app-builder:v1

[packages]
"@aica/base" = "v3.1.1"

# add components
"@aica/components/signal-processing" = "v1.0.1"

# add hardware collections 
"@aica/collections/ur-collection" = "v3.0.5"
```

### Including custom packages

The AICA framework allows developers to build their
own [custom components](../reference/custom-components/01-component-package.md). These packages can be included under
a custom name using the `docker-image://` prefix to specify the docker image name or path. For example, a custom
component package that was locally built using `docker build [...] --tag my-custom-component-package` could be included
as `docker-image://my-custom-component-package`. Community and third-party packages may also be available on other
docker registries such as DockerHub or GitHub Container Registry and can be included with the associated docker path.

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/app-builder:v1

[packages]
"@aica/base" = "v3.1.1"

# add a custom package from a local docker image path
"my-local-package" = "docker-image://my-custom-component-package"

# add a package from any docker path such as GitHub Container Registry
"my-ghcr-package" = "docker-image://ghcr.io/user/package:tag"
```

## Building an AICA runtime application image

:::note

[Log in to the package registry](#logging-in-to-the-aica-package-registry) before building the image to authorize docker
to access AICA packages.

:::

Once the desired packages have been configured in a manifest file, a `docker build` command can be used to build the
runtime application image. In this example, a manifest file saved as `aica-package.toml` is used to build an image
with the name `aica-runtime`.

```shell
docker build -f aica-package.toml -t aica-runtime .
```

The command `docker image ls | grep aica-runtime` should then list the `aica-runtime` image.

Continue to the next section to learn how to start the application container.
