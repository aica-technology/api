---
title: Proxies
---

import DockerDesktopProxies from './assets/docker-desktop-proxies.png'

AICA programs sometimes need Internet connectivity to function. This section details the services that must be allowlisted on your proxy for each and how to configure your computer to use it.

## Services

The following services must be allowlisted on your proxy for AICA programs to function:

AICA Core (Online License):

- `licensing.aica.tech` / `api.licensing.aica.tech` (HTTPS, 443): used to check the license & authenticate Cloud Storage
- `ingress-api.analytics.aica.tech` (HTTPS, 443): used to send analytics data
- `api.storage.aica.tech` (HTTPS, 443): used to access your Cloud Storage

AICA Launcher:

- `raw.githubusercontent.com` (HTTPS, 443): used to fetch Terms of Use and the OSS packages manifest
- `ghcr.io` (HTTPS, 443): used to download OSS packages
- `registry.licensing.aica.tech` (HTTPS, 443): used to download licensed packages
- `api.licensing.aica.tech` (HTTPS, 443): used to authenticate to the licensed packages registry

## Configuration

### Docker

As our products rely on Docker, you will need to configure your Docker daemon to use your proxy.

Check [this guide](https://docs.docker.com/engine/cli/proxy/) for instructions on how to configure your Docker daemon to use your proxy.

This can also be done through the UI if using Docker Desktop.

<div class="text--center">
  <img src={DockerDesktopProxies} alt="Docker Desktop settings showing the proxies configuration under Resources > Proxies" />
</div>

### AICA Launcher

AICA Launcher supports the following environment variables for proxy configuration:

- HTTP: `HTTP_PROXY` or `http_proxy`
- HTTPS: `HTTPS_PROXY` or `https_proxy`
- Bypass: `NO_PROXY` or `no_proxy`
- CA Certificate (used to launch AICA Core): `SSL_CERT_FILE` or `ssl_cert_file`

Starting with Launcher v2.1, those settings will be properly forwarded to AICA Core instances started through the app.

<details>
<summary>MacOS</summary>

You need to use `launchctl setenv` to set environment variables that can be used by applications started through Finder.

```bash
launchctl setenv HTTP_PROXY "$HTTP_PROXY"
launchctl setenv HTTPS_PROXY "$HTTPS_PROXY"
```

This is not required if you open AICA Launcher from the command line using `open`.

</details>

### AICA Core

:::tip

Starting with Launcher v2.1, proxy settings are setup automatically for AICA Core.

:::

Starting with v5.0, AICA Core supports the following environment variables for proxy configuration:

- HTTP: `HTTP_PROXY` or `http_proxy`
- HTTPS: `HTTPS_PROXY` or `https_proxy`
- Bypass: `NO_PROXY` or `no_proxy`

You must provide the environment variables to AICA Core when starting the container. If you choose to intercept HTTPS traffic, you will also need to mount your certificate inside `/etc/ssl/certs`.

Example:

```bash
docker run %ANY_OTHER_DOCKER_OPTIONS% -e HTTP_PROXY -e HTTPS_PROXY -e NO_PROXY -v /where/is/my/proxy/ca_cert.pem:/etc/ssl/certs/myproxy.pem my-core
```

<details>
<summary>MacOS</summary>

If you use a local proxy on macOS, you will need to change the host from `localhost` or `127.0.0.1` to `host.docker.internal`.

Example: `-e HTTP_PROXY=http://localhost:8000` becomes `-e HTTP_PROXY=http://host.docker.internal:8000`

</details>
