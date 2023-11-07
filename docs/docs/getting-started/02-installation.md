---
sidebar_position: 2
title: Installation
---

## Pre-requisites

AICA software is distributed with Docker images and executed with Docker containers.

Before proceeding, be sure to [install Docker Engine](https://docs.docker.com/engine/install/) on the host machine.

## Licensing

A valid license is required to use AICA software. A license regulates both the access rights to download AICA software
packages and the usage rights to run AICA applications.

To request a license, contact the AICA sales team at contact@aica.tech. A user-specific license key will be generated
and sent by email. For the rest of this guide, it will be assumed that a valid license has been saved to a file called
`aica-license.toml` on the host machine.

AICA licenses come in two variants: **online** and **offline**.

### Online licenses

:::info

- Online licenses require an active internet connection for the entire duration of the application.
- Online licenses can be used on any machine, but only one instance can be running at any one time.

:::

An online license will appear in the following format, though the specific license key will be unique for each user.

```toml title="aica-license.toml"
License = "5614D1-3E7A6C-932DEB-8C4189-F6B0F2-V3"
```

### Offline licenses

:::info

- Offline licenses can be used without an active internet connection.
- Offline licenses must be registered and activated on a specific machine, and can only be used on that machine.
- Offline licenses are a premium service and are available on request for customers with strict network limitations or
  requirements.

:::

To prepare a target machine for an offline license, run the following command to identify its unique machine
fingerprint.

```shell
docker run --privileged --rm ghcr.io/aica-technology/machine-fingerprint
```

If it is not possible to temporarily provide network access to the target machine, first pull and save
the `ghcr.io/aica-technology/machine-fingerprint` utility docker image on an online machine, then transfer and load it
on the target machine:

```shell
# download the machine fingerprint helper an online machine
user@online-machine:~$ docker pull ghcr.io/aica-technology/machine-fingerprint
user@online-machine:~$ docker save ghcr.io/aica-technology/machine-fingerprint > machine-fingerprint.tar

# copy the machine-fingerprint.tar file to the offline machine, then load and run it
user@offline-machine:~$ docker load < machine-fingerprint.tar
user@offline-machine:~$ docker run --privileged --rm ghcr.io/aica-technology/machine-fingerprint
```

Executing this command should print out a long string of characters containing a machine-specific fingerprint. Copy the
output of the command and send it to the established contact person in the AICA support team. They will then generate
and send a unique license key which contains the encrypted machine fingerprint. It will appear as a long string in the
following format:

```toml title="aica-license.toml"
License = "key/eyJ9df2jfap7IVdIHnlnNpb24[...]alSBR_tBSIjavblcziV5nBQ=="
```

## Logging in to the AICA package registry

AICA software packages are hosted in a private container registry. Use a valid license file to authenticate docker to
login and pull images from the registry using the following command:

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
#syntax=ghcr.io/aica-technology/app-builder:v1.0.0

[packages]
"@aica/base" = "v2.0.0"
```

### Configuring a runtime image with add-on packages

```toml title="aica-package.toml"
#syntax=ghcr.io/aica-technology/app-builder:v1.0.0

[packages]
"@aica/base" = "v2.0.0"

# add components
"@aica/components/gear-assembly" = "v2.0.0"

# add hardware collections 
"@aica/collections/ur-collection" = "v2.0.0"
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
