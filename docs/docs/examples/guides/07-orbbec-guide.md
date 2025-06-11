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

AICA's `hardware_collection` package now includes support for the Orbbec camera series, including the Gemini 33X models, Gemini 435Le, Gemini 2/2L, Femto Bolt/Mega and Astra 2. This guide will describe all necessary steps to install, configure and run the component, using the AICA Studio.  

## Installation

To ensure proper access to the camera device, special udev rules have to be applied to the Linux system the camera is connected to. Simply run the following commands in a terminal: 

```shell
wget -O 99-obsensor-libusb.rules https://raw.githubusercontent.com/aica-technology/OrbbecSDK_ROS2/refs/heads/main/orbbec_camera/scripts/99-obsensor-libusb.rules
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

These commands will download and apply the udev rules. To ensure that the rules have been applied, you can use the [Orbbec Viewer](https://github.com/orbbec/OrbbecSDK/releases) to see the camera feed.

In order to also ensure the camera can be accessed from within the container, you need to mount the /run/udev/ volume. In the AICA Launcher window, expand the **Advanced Settings** in the bottom, select **Add a volume mount** and select the volume, as shown in the following image:

<div class="text--center">
  <img src={orbbecMountedVolume} alt="Mounted udev volume" />
</div>

Select **Launch AICA Studio** to proceed. 

## Creating and running an application

Let us start by creating a new application. By default, the new application contains a hardware interface, which you cna later use to also include your robot. For now, it is not necessary, so you can remove it. Click on the Add button on the top right, and locate the Orbbec camera component. Select to add to the workspace. Finally, connect the start block to the component.

**TODO:** Will the component appear as it shows in the GIF, or under Core? 

<div class="text--center">
  <img src={orbbecNewApp} alt="Creating a frame in AICA Studio" />
</div>

Select **Play** to start the application. Once the component appears loaded, click on the gear icon on the bottom right and select **Launch RViz**. This will allow us to see the live camera feed. Onece RViz launches, select Add > By topic > /color_image > Image. This adds a panel that shows the live color image. You can also find the depth image under /depth_image > Image.

<div class="text--center">
  <img src={orbbecRvizColor} alt="Creating a frame in AICA Studio" />
</div>



<!-- 1. Open the settings menu of the hardware interface and set the URDF to `Generic six-axis robot arm`, then
proceed to close this menu.
2. Add a `Joint Trajectory Controller` to the hardware interface and set it to:
   - auto-load
   - auto-activate
3. Connect the start block to the hardware interface to load it on start. -->


## Parametrizing Orbbec camera

Click on the small gear icon on the `Orbbec Camera` block we just added to view and edit the available
parameters.

<div class="text--center">
  <img src={orbbecCameraParameters} alt="Basic Orbbec camera parameters" />
</div>

You can hover over the exclamation marks next to the names to see a detailed decription. We will explain some of these here as well:

- `Color/Depth Width/Height/FPS`: these refer to the resolution and frame rate of the color and depth images. Keep in mind that only specific pairs of numbers apply here. For more information check your camera's documentation. The Orbbec Viewer can also come in handy here, as you can use it to see what profiles are available.
- `Enable Alignment`: boolen that activates the spatial alignment of the depth image to the corresponding color image. Generates a depth image with the same size as the colour, with depth expressed in the color camera coordinate system. 
- `Enable Threshold Filter`: boolean that activates a threshold filter in the depth image. Activation of the filter allows the optional definition of some additional parameters -if they are not defined, the default values will be used. To see what parameters you can define, hover over the exclamation mark. To avoid cluttering the parameter tab of the component, these parameters can only be defined in the YAML. Activate the filter, and set them as follows. Your YAML component section should be as follows:

<div class="text--center">
  <img src={orbbecThresholdParameters} alt="Threshold tooltip parameters" />
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

Click on **Generate Graph** to make the changes take effect. Start the application, launch RViz and add a panel for the depth image. Only objects between 0.5 and 2 meters should appear. Keep in mind that most cameras have a minimum Z depth; if you set the maximum threshold below this value, the image will appear blank. Follow the same process to activate and define the rest of the filters.  