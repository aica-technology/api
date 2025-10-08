---
sidebar_position: 9
title: ABB manipulators
---

import abbRSSuccessfulConnection from './assets/abb-successful-connection.png'
import abbNewUDPUCDevice from './assets/abb-new-udpuc-device.png'
import abbControllerConfiguration from './assets/abb-controller-configuration.png'
import abbAdditionalOptions from './assets/abb-additional-options.png'
import abbNewProject from './assets/abb-new-project.png'
import abbInstallAddins from './assets/abb-install-addins.png'
import abbConnectController from './assets/abb-connect-controller.png'
import abbFirewallManager from './assets/abb-firewall-manager.png'
import abbHIParameters from './assets/abb-hi-parameters.png'

# ABB manipulators

ABB offers a wide range of industrial articulated manipulators, from compact 6-axis robots for small-part handling and
payloads of a few kilograms to heavy-duty models capable of lifting up to 800 kg. This guide provides instructions for
using ABB robots within the AICA System, focusing on connecting and configuring both simulated environments using
RobotStudio and real hardware setups.

To use the ABB collection, add the latest version of `collections/abb` to your configuration in AICA Launcher, currently
supporting the following robot models out of the box:

- IRB 1010
- GoFa CRB 15000, 12 Kg

## General

This guide and the provided drivers are designed for ABB robots connected to OmniCore controllers, whether in simulation
or with real hardware. Ensure your setup uses **RobotWare for OmniCore** for full compatibility with the instructions
and features described below.

:::warning

This collection supports RobotWare versions 7.X and above. For older versions, contact the AICA support team.

:::

### Externally Guided Motion

ABB permits remote control of its manipulator range using the Externally Guided Motion (EGM) feature. EGM
provides external devices with the ability to send commands and control ABB robotic arms, using Google's Protocol
Buffers (Protobuf) serialization library to transport information through UDP sockets. For more information, check out
the
[official product documentation](https://library.e.abb.com/public/344f15f0f43341eb944fe35279d9fa2e/3HAC073319+AM+Externally+Guided+Motion+RW6-en.pdf?x-sign=WlxgV7Vao27KV3d3hlsfaoykgctYqoA0F98ch89S%2FPEaGwQg47ou%2FioylQtzvLaV). 

:::warning

EGM is an optional add-in and has to be purchased separately. 

:::

### Robot Web Services

The second ABB feature that AICA System utilizes to connect to the robot is Robot Web Services (RWS). RWS is a platform
that enables developers to create applications that interact with the robot controller, using RESTful APIs that leverage
the HTTPS protocol. The hardware interface uses RWS for auxiliary functionality, such as starting/stopping the program
and the motors, and setting IOs. Setting up RWS on the simulator and on the actual robot requires slightly different steps,
which will be explained in the following sections. More information can be found on the
[product reference page](https://developercenter.robotstudio.com/api/RWS).

### RAPID

RAPID is the programming language of ABB robots. Users can utilize RAPID to set up and execute their workflows and
processes. This is enabled by user-defined libraries called *Modules*, that contain variables and functions or
processes (*PROCs*). Modules can then be loaded in controller *Tasks*, and called as required.

## Connecting to a robot

The hardware interface provided by the ABB collection can be used to control either a simulated or a real robot. The
following sections provide the necessary steps to do both.

### RobotStudio simulation

[RobotStudio](https://new.abb.com/products/robotics/software-and-digital/robotstudio) is the official ABB offline
programming and simulation tool for robotics applications. It allows to run virtual controllers that mimic the behavior
of the real robot, ensuring seamless transition between simulation and hardware. It can also be used to configure
several aspects of the real controller.

:::note

The RobotStudio software suite is available only for Windows, so you may need to set it up on a secondary device or a
virtual machine .

:::

Setting up a virtual workstation and controller can be achieved by following the next steps:

1. In RobotStudio, navigate to the **Add-Ins** tab and go to **Gallery**. There is a list of all available robot models
   and RobotWare versions, the internal controller software. To ensure consistency between simulation and reality, make 
   sure to install the versions matching the real robot controller, if one is available.
   <div class="text--center">
     <img src={abbInstallAddins} alt="Install necessary addins in RobotStudio." />
   </div>
2. Go back to to **File > New > Project**.
3. Select to create a new controller and define the robot model and variant, as well as the RobotWare version.
4. Make sure to activate the **Customize Options** button. This is required to add EGM in a next step.
   <div class="text--center">
     <img src={abbNewProject} alt="Create a new RobotStudio project." />
   </div>
5. Select **Create** to create the new project.
6. In the window that pops up, in the **Options** tab, look for **EGM** and **RobotStudio Connect** and add them in the
   controller. Then select **Apply and Reset** to finalize.
   :::note
   The **3119-1 RobotStudio Connect** add-in is required to connect a controller to RobotStudio over a public network. For more
   information, see the RobotStudio instruction manual. 
   :::
   <div class="text--center">
     <img src={abbAdditionalOptions} alt="Additional options in the RobotStudio project." />
   </div>

:::tip

Another way of ensuring applications are set up correctly is to use the provided mock interfaces. The mock interface 
does not include a physics engine, or dynamics calculations, and merely sets the commands as state.

::: 

### Real robot controller

RobotStudio can be used to configure the address of the external control device. Navigate to the Controller tab and
select **Add Controller > Connect to Controller**. This will allow to detect and connect to the running controller in
OmniCore, provided of course that the devices are on the same network.

<div class="text--center">
  <img src={abbConnectController} alt="Connect to the robot controller." />
</div>

## Network and device configuration

:::tip

Modifications in the real robot controller require write access. To get it, click on **Request Write Access** and
confirm on the pendant's screen. To save these modifications, the controller has to be restarted. While this takes
seconds in simulation, the restart procedure in the real robot might take a few minutes, so make all necessary changes,
and then restart.

:::

After connecting to the robot, the controller should be configured to accept commands from an external device.

1. Navigate to the Controller tab > Configuration > Communication > UDP Unicast Device, and add a new UDPUC device (or
   modify the existing one), configured as shown below. This is the device that will be running the AICA application,
   the external control device, so the address should be set accordingly.
   <div class="text--center">
     <img src={abbControllerConfiguration} alt="Controller configuration settings." />
   </div>
   <div class="text--center">
     <img src={abbNewUDPUCDevice} alt="Add a new UDPUS device." />
   </div>
2. The next step is enabling RWS connection. For the RobotStudio simulation, this requires either using a proxy or
   whitelisting the IP address of the device trying to access RWS, which is the device running AICA Core.
   The first approach is preferable and described analytically in a
   [RobotStudio forum post](https://forums.robotstudio.com/discussion/12082/using-robotwebservices-to-access-a-remote-virtual-controller)
   (read until the end and the last comment for a critical fix).
   :::note
   OmniCore controllers and RobotWare 7.x versions by default listen on HTTPS and port 80 for RobotStudio and 443 for
   the real robot. If necessary, the port numbers can be modified by following the instructions in this
   [forum post](https://forums.robotstudio.com/discussion/12177/how-to-change-the-listening-port-of-the-virtual-controller-robotware-6-x-and-7-x).
   :::
   <!-- TODO: write down actual commands -->
3. (Only for connection with the RobotStudio simulation) To communicate with the RWS running in the Windows device,
   the firewall in the respective network (usually Public) needs to be deactivated.
4. In the firewall settings of RobotStudio, make sure that UDPUC and RobotWebServices are enabled in the network that is
   being used (see picture below). Finally, for the changes to take effect, restart the controller.
   <div class="text--center">
     <img src={abbFirewallManager} alt="Firewall manager options." />
   </div>

## RAPID module

The ABB hardware interface needs a suitable RAPID module that uses EGM to allow external control to be running on the
robot. An example of such a module is shown below. It's important to make sure that the UCDevice name used in the RAPID
module corresponds to the UDPUC device configured during step 1 above. The module can be placed in the controller's home
 directory and uploaded to the current task.

<details>
  <summary>Example RAPID module</summary>

```bash
MODULE AICAMain
  !***********************************************************
  ! Program data
  !***********************************************************
  ! Home position.
  LOCAL VAR jointtarget current;

  ! Identifier for the EGM correction.
  LOCAL VAR egmident egm_id;

  ! Limits for convergance.
  LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

  !***********************************************************
  !
  ! Procedure main
  !
  !   This RAPID code exemplifies how to run EGM with velocity control
  !
  !   Note: Update the UCDevice "AICA_PC" with correct
  !         values for the remote address and port
  !         (i.e. to the EGM server).
  !
  !         Update via RobotStudio:
  !         Controller tab -> Configuration ->
  !         Communication -> UDP Unicast Device
  !
  !***********************************************************
  PROC main()
      current := CJointT();
      MoveAbsJ current, v200, fine, tool0;

      ! Register an EGM id.
      EGMGetId egm_id;

      ! Setup the EGM communication.
      EGMSetupUC ROB_1, egm_id, "default", "AICA_PC", \Joint;

      ! Prepare for an EGM communication session.
      EGMActJoint egm_id
                  \J1:=egm_condition
                  \J2:=egm_condition
                  \J3:=egm_condition
                  \J4:=egm_condition
                  \J5:=egm_condition
                  \J6:=egm_condition
                  \MaxSpeedDeviation:=120.0;

      WHILE TRUE DO
          ! Start the EGM communication session.
          EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=5 \RampOutTime:=5 \PosCorrGain:=0;
          EGMStop egm_id, EGM_STOP_HOLD \RampOutTime:=5;
      ENDWHILE
      ! Release the EGM id.
      EGMReset egm_id;


  ERROR
      IF ERRNO = ERR_UDPUC_COMM THEN
          TPWrite "Communication timed out";
          TRYNEXT;
      ENDIF
  ENDPROC
ENDMODULE
```

</details>

## Hardware interface

:::tip

For best results, always set the rate of the hardware interface to 250 Hertz, which corresponds to the ABB-suggested
stable UDP data exchange limit.

:::

Returning to AICA Studio and the hardware interface, it is now possible to define the parameters and connect to the
robot. The majority of the hardware interface parameters enable connection to EGM and RWS:

<div class="text--center">
  <img src={abbHIParameters} alt="ABB Hardware interface parameters." />
</div>

- EGM port: the port that EGM uses to send commands (6511, the Remote port of the UDPUC device defined in RobotStudio).
- RWS IP & port: address and port of the RWS (the address of the RobotStudio device or real robot).
- Connection timeout: for RWS connection.
- Settling time constant: the time within which the robot will reach a certain percentage of the target position (only 
  used for position commands). Lower values make the motion faster and the robot more responsive.  
<!-- - Rapid File Path: the path for the module to be loaded in the controller home directory. -->
<!--- Headless Mode: if true, the hardware interface handles the whole initialization procedure of starting and stopping the
  RAPID program and the motors without requiring any user interaction. -->

Running the application can successfully connect to the robot and get information about the setup. Starting the motors and
the RAPID program must be done manually through the pendant. After that, the robot is ready to receive commands. 

<!-- <div class="text--center">
  <img src={abbRSSuccessfulConnection} alt="Connected to RobotStudio successfully." />
</div> -->
