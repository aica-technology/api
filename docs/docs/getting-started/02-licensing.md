---
sidebar_position: 1.5
title: Licensing
---

# Licensing

A valid license is required to use AICA software. A license regulates both the access rights to download AICA software
packages and the usage rights to run AICA applications.

To request a license, contact the AICA sales team at contact@aica.tech. A user-specific license key will be generated
and sent by email. For the rest of this guide, it will be assumed that a valid license has been saved to a file called
`aica-license.toml` on the host machine.

:::caution

Your license key should be kept secret. Do not share your license key with unauthorized users or enter it on websites
other than the official AICA domain. License abuse may prevent your application from running or lead to your license
being revoked.

If an unauthorized user has gained access to your license key, contact support@aica.tech to reset your license.

:::

AICA licenses come in two variants: **online** and **offline**.

## Online licenses

:::info

- Online licenses require an active internet connection for the entire duration of the application.
- Online licenses can be used on any machine, but only one instance can be running at any one time.

:::

An online license will appear in the following format, though the specific license key will be unique for each user.

```toml title="aica-license.toml"
License = "5614D1-3E7A6C-932DEB-8C4189-F6B0F2-V3"
```

## Offline licenses

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

---

Continue to the next section to learn how to use the license file to access and install AICA packages with Docker.
