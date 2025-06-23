---
sidebar_position: 4
title: Installation on Windows
---

import winFeatures from './assets/win-features.png';
import winHyperV from './assets/win-hyper-v.png';
import wslInstall from './assets/wsl-install.png';
import dockerSettings from './assets/win-docker-settings.png';

# Installation on Windows

The AICA System can be installed and run on Windows by leveraging **WSL**, short for
[Windows Subsystem for Linux](https://learn.microsoft.com/en-us/windows/wsl/about). WSL is a feature of Windows that
allows running a Linux environment on a Windows machine without the need for a separate virtual machine or dual booting.
While most Linux distributions can be run with either WSL 1 or WSL 2 architecture, WSL 2 has superior performance over
WSL 1 and will be assumed throughout these instrucations.

On a high level, there are two main requirements for running the AICA System on Windows:

- Ubuntu 22.04 or 24.04 installed through WSL 2
- Docker Desktop that uses the WSL 2 based engine

:::note

The following instructions for running the AICA System have been tested and validated on a fresh installation of Windows
11 Professional with full administrator access. Depending on the machine at hand, some steps might be slightly different
or extended access might be required. For questions, consult the
[official documentation](https://learn.microsoft.com/en-us/windows/wsl/install-manual), contact your IT support, or
reach out to the AICA team.

:::

## Setup steps

1. Ensure that the Hyper-V feature is on. Open the Windows Features from the Control panel, locate Hyper-V and check the
   box if necessary. Switching it on requires a restart of the system. Alternatively,
   [use the terminal](https://learn.microsoft.com/en-us/windows/wsl/install-manual#step-3---enable-virtual-machine-feature)
   to enable it.

    <div style={{ display: "flex", gap: "16px", justifyContent: "center" }}>
        <img src={winFeatures} alt="Windows Features" width="45%" />
        <img src={winHyperV} alt="Enable Hyper-V" width="45%" />
    </div>

2. [Optional]
   [Download the Linux kernel update package](https://learn.microsoft.com/en-us/windows/wsl/install-manual#step-4---download-the-linux-kernel-update-package).

3. Open PowerShell or Windows Command Prompt in **administrator** mode and run

   ```shell
   wsl --install
   ```

   The installation procedure will prompt for a username and password. The result of this step should look like this:

   <div class="text--center">
     <img src={wslInstall} alt="Install WSL" />
   </div>

   Verify that the installed Ubuntu version is either 22.04 or 24.04 by typing `lsb_release -a` in the terminal above.
   If that's not the case, consult
   [this page](http://learn.microsoft.com/en-us/windows/wsl/install#change-the-default-linux-distribution-installed) to
   find out how to install a different Ubuntu distribution with WSL.

4. Restart your machine for the changes to take effect.

5. Install Docker Desktop and enable WSL 2 backend by following
   [steps 1 to 6 of this guide](https://docs.docker.com/desktop/features/wsl/#turn-on-docker-desktop-wsl-2). After
   restarting Docker Desktop, navigate to the settings and verify that the WSL 2 based engine is enabled:

   <div class="text--center">
     <img src={dockerSettings} alt="Install WSL" />
   </div>

6. Launch Ubuntu either with the icon from the start menu or by entering `wsl` in a Windows terminal. Verify the Docker
   installation by running `docker run hello-world`.
