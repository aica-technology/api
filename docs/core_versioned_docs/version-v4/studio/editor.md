---
sidebar_position: 2
title: Application editor
---

# Application editor

When an application is created or selected for editing, the following application editor screen will appear.

![aica-studio-application-editor](./assets/aica-studio-application-editor.png)

On the left of the screen is the YAML code representation of the application, which is how applications are stored and
parsed. On the right are visualization tools, including the unique application graph editor and the 3D scene viewer.

The white chevrons in the middle of the screen can be used to selectively expand the left or right portions of the
screen.

Application controls are shown above the YAML code.

Exit back to the application manager screen using the X button on the top left. It will prompt to save or discard
unsaved changes. Running applications will be stopped when closing the application.

Next to the X button, the cloud storage button can be used to upload and synchronize applications to cloud storage or
downloaded them locally. This option is disabled until a user with `admin` scope connects to cloud storage in the
[Settings](./studio.md#settings) page.

Applications can be deleted, saved, or saved as a copy under a new name.

The "Generate Graph" button is used to apply new code changes to the application graph. When editing the YAML code
manually, changes will not take effect until this button is pressed.

The application can be renamed using the edit icon next to the application name. This also presents an option to add or
edit the application description, which can serve as a way of differentiating and describing the purpose of various
applications.

In the top right of the application editor are the runtime controls. The Play button starts the loaded application,
which can then be stopped with the Stop button.

Beneath the runtime controls are the view selections for the various visualization tools, starting with the application
graph. These views are presented in more detail in the next section.
