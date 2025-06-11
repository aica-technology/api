---
sidebar_position: 5
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

import orbbecMountedVolume from './assets/orbbec-mounted-udev-volume.png'
import orbbecNewApp from './assets/orbbec-new-app.gif'
import orbbecRvizColor from './assets/orbbec-play-rviz.gif'
import orbbecCameraParameters from './assets/orbbec-camera-parameters.png'
import orbbecThresholdParameters from './assets/orbbec-threshold-params.png'


# Orbbec camera example

**TODO:** Define a release version, AICA Core version or something similar?

AICA's `hardware_collection` package now includes support for the Orbbec camera series, including the
Gemini 33X models, Gemini 435Le, Gemini 2/2L, Femto Bolt/Mega and Astra 2. This guide will describe all
necessary steps to install, configure and run the component, using the AICA Studio.  

## Installation

To ensure proper access to the camera device, special udev rules have to be applied to the Linux system
the camera is connected to. Simply run the following commands in a terminal: 

```shell
wget -O 99-obsensor-libusb.rules https://raw.githubusercontent.com/aica-technology/OrbbecSDK_ROS2/refs/heads/main/orbbec_camera/scripts/99-obsensor-libusb.rules
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

These commands will download and apply the udev rules. To ensure that the rules have been applied
correctly, the [Orbbec Viewer](https://github.com/orbbec/OrbbecSDK/releases) can be used to check 
the camera feed.

In order to also ensure the camera can be accessed from within the container, the _/run/udev/_ volume
has to be mounted. In the AICA Launcher window, expand the **Advanced Settings** in the bottom, select
**Add a volume mount** and define the volume, as shown in the following image:

<div class="text--center">
  <img src={orbbecMountedVolume} alt="Mounted udev volume" />
</div>

Select **Launch AICA Studio** to proceed. 

## Creating and running an application

Start by creating a new application. 

1. By default, the new application contains a hardware interface, which can later be used to also
connect and include a robot. For now, it is not necessary, so it can be removed. 
2. Click on the Add button on the top right, and locate the **Orbbec Camera** component. Click to add
to the workspace. 
3. Finally, connect the start block to the component.

**TODO:** Will the component appear as it shows in the GIF, or under Core? 

<div class="text--center">
  <img src={orbbecNewApp} alt="Creating a new Orbbec Camera component" />
</div>

4. Select **Play** to start the application. 
5. Once the component appears loaded, click on the gear icon on the bottom right and select
**Launch RViz**. This will allow to check the live camera feed. 
6. Once RViz launches, select _Add > By topic > /color_image > Image_. This adds a panel that shows the
live color image. The depth image can also be found under _/depth_image > Image_.

<div class="text--center">
  <img src={orbbecRvizColor} alt="Starting and checking camera live stream" />
</div>

## Parametrizing Orbbec camera

Click on the small gear icon on the `Orbbec Camera` block to view and edit the available parameters.

<div class="text--center">
  <img src={orbbecCameraParameters} alt="Basic Orbbec camera parameters" />
</div>

Hovering over the exclamation marks next to the names shows a detailed decription of the parameter. 
Let's explain some of these here:

- `Color/Depth Width/Height/FPS`: these refer to the resolution and frame rate of the color and depth
images. Keep in mind that only specific pairs of integer values apply here. For more information 
check the camera's documentation. The Orbbec Viewer can also come in handy here, as it can be used 
to see what profiles are available.
- `Enable Alignment`: boolean that activates the spatial alignment of the depth image to the
corresponding color image. Generates a depth image with the same size as the color, with depth
expressed in the color camera coordinate system. 
- `Enable Threshold Filter`: boolean that activates a threshold filter in the depth image. Activation
of the filter allows the optional definition of some additional parameters -if they are not defined, the
default values will be used. To see what parameters can be defined, hover over the exclamation mark. To 
avoid cluttering the parameter tab of the component, these parameters can only be defined in the YAML.
Activate the filter and set them so the YAML component section is as follows:

<div class="text--center">
  <img src={orbbecThresholdParameters} alt="Threshold filter tooltip parameters" />
</div>


```yaml
components:
  orbbec_camera:
    component: orbbec_camera::OBCameraNodeDriver
    display_name: Orbbec Camera
    parameters:
      enable_threshold_filter: true
      threshold_filter_max: 2000
      threshold_filter_min: 500
```

Click on **Generate Graph** to make the changes take effect. Start the application, launch RViz and add
a panel for the depth image. Only objects between 0.5 and 2 meters should appear. Keep in mind that most
cameras have a minimum Z depth; if the maximum threshold is set below this value, the image will appear
blank. For more information about that, check the camera's documentation. 

Follow the same process to activate and define the rest of the filters.  