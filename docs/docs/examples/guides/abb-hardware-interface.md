---
sidebar_position: 10
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

# ABB manipulators

ABB offers a wide range of industrial articulated manipulators, from compact 6-axis robots for small-part handling and
payloads of a few kilograms to heavy-duty models capable of lifting up to 800 kg. This guide aims to support the
integration of ABB robots in AICA System applications, from the mock interface and a simulated environment to the real
robot.

To use the ABB collection, add the latest version of `collections/abb` to your configuration in AICA Launcher, currently
supporting the following robot models:

- IRB 1010
- GoFa CRB 15000, 12 Kg

**TODO: Where are we mentioning the RWS controller, if at all?**

## General

In this section, some of the prerequisites to use the AICA ABB collection are introduced and discussed.

### EGM

ABB permits remote control of its manipulator range using the optional **Externally Guided Motion (EGM)** feature. EGM
provides external devices with the ability to send commands and control ABB robotic arms, using Google Protocol Buffers
transported through UDP sockets. For more information, check out the
[official product documentation](https://library.e.abb.com/public/344f15f0f43341eb944fe35279d9fa2e/3HAC073319+AM+Externally+Guided+Motion+RW6-en.pdf?x-sign=WlxgV7Vao27KV3d3hlsfaoykgctYqoA0F98ch89S%2FPEaGwQg47ou%2FioylQtzvLaV).

:::tip

For best results, always set the rate of the hardware interface to 250 Hertz, which corresponds to the suggested stable
UDP data exchange limit.

:::

### RWS

The second ABB feature that AICA system utilizes to connect to the robot is RobotWebServices (RWS). RWS is a platform
that enables developers to create applications that interact with the robot controller, using RESTful APIs that leverage
the HTTPS protocol. The hardware interface uses RWS for auxiliary functionality, such as starting/stopping the program
and the motors, and setting IOs. Setting up RWS in the simulator and the actual robot requires slightly different steps,
which will be explained in the following sections. More information can be found in the
[product reference page](https://developercenter.robotstudio.com/api/RWS).

### RAPID

RAPID is the programming language of ABB robots. Users can utilize RAPID to set up and execute their workflows and
processes. This is enabled by user-defined libraries called **Modules**, that contain variables and functions or
processes **(PROCs)**. Modules can then be loaded in controller **Tasks**, and called as required.

<!-- :::note

The example above drives the robot in velocity by setting the **PosCorrGain** parameter to 0 in the control loop. Remove
this if the robot needs to receive position commands.

::: -->

## Mock interface

You can use the ABB manipulator mock interface variant to check the application execution, visualize trajectory
execution and ensure correct frame definition, among else. The mock interface is a very useful tool during early
development stages, as it does not require connection to the actual hardware and provides a safe environment to
experiment. After correct operation is verified, users can just switch to the real hardware interface to connect to the
actual robot or simulator.

:::note

The mock interface is only a visualization tool and does not include a physics engine, or dynamics calculations. For
that purpose, check out RobotStudio in the following section.

:::

## Connecting to a robot

The provided hardware interface can be used to connect and control either a simulated or a real robot. The following 
sections provide the necessary steps to connect to both. 

### RobotStudio simulation

[RobotStudio](https://new.abb.com/products/robotics/software-and-digital/robotstudio) is the official ABB offline
programming and simulation tool for robotics applications. It allows to run virtual controllers that mimic the behavior
of the real robot, ensuring seamless transition between simulation and hardware. It can also be used to configure
several aspects of the real controller.

:::note

The RobotStudio software suite is available for Windows so it would require a second device.

:::

Setting up a virtual workstation and controller is the next step after using the mock interface. This can be achieved by
following the next steps:

1. In RobotStudio, navigate to the **Add-Ins** tab and go to **Gallery**. There is a list of all available robot models
   and RobotWare versions, the internal controller software. Make sure to install the versions present in the actual
   robot controller, to ensure consistency between simulation and reality.

<div class="text--center">
  <img src={abbInstallAddins} alt="Install necessary addins in RobotStudio." />
</div>

2. Go back to to File > New > Project.
3. Select to create a new controller and define the robot model and variant, as well as the RobotWare version.
4. Make sure to activate the **Customize Options** button. This is required to add EGM in a next step.

<div class="text--center">
  <img src={abbNewProject} alt="Create a new RobotStudio project." />
</div>

5. Select **Create** to create the new project.
6. In the window that pops up, in the **Options** tab, look for **EGM** and **RobotStudio Connect** and add them in the
   controller. Then select **Apply and Reset** to finalize.

<div class="text--center">
  <img src={abbAdditionalOptions} alt="Additional options in the RobotStudio project." />
</div>


### Omnicore controller & real robot

RobotStudio can be used to configure the address of the external control device. Navigate to the Controller tab and
select **Add Controller > Connect to Controller**. This will allow to detect and connect to the running controller in
Omnicore, provided of course that the devices are on the same network. 

<div class="text--center">
  <img src={abbConnectController} alt="Connect to the robot controller." />
</div>

## Network and device configuration

:::note

Modifications in the real robot controller require write access. To get it, click on **Request Write Access** and confirm on the
pendant's screen.

:::

After connecting to the robot, the controller should be configured to accept commands from an external device.
Navigate to the Controller tab > Configuration > Communication > UDP Unicast Device, and add a new UDPUC device (or
modify the existing one), configured as shown below. This is the device that will be running the AICA application, the
external control device, so the address should be set accordingly. Finally, for the changes to take effect, you need to
restart the controller.

<div class="text--center">
  <img src={abbControllerConfiguration} alt="Controller configuration settings." />
</div>

<div class="text--center">
  <img src={abbNewUDPUCDevice} alt="Add a new UDPUS device." />
</div>

:::tip

While this takes seconds in simulation, the restart procedure in the real robot might take a few minutes, so make all necessary 
changes and then restart.

:::

Next step is enabling RWS connection. For the simulation specifically, this requires either using a proxy or whitelisting the IP address of the device
trying to access RWS, in this case the device running the AICA application. The first approach is preferable and
described analytically in a
[RobotStudio forum post](https://forums.robotstudio.com/discussion/12082/using-robotwebservices-to-access-a-remote-virtual-controller)
(read until the end and the last comment for a critical fix). Omnicore controllers and RobotWare 7.x versions by default
listen on HTTPS and port (80 for RobotStudio and 443 for the real robot). This can be modified by following the instructions in this
[forum post](https://forums.robotstudio.com/discussion/12177/how-to-change-the-listening-port-of-the-virtual-controller-robotware-6-x-and-7-x). 
Additionally, to communicate with the RWS running in the Windows device, the firewall in the respective network (usually Public) needs
to be deactivated.

Next, make sure that UDPUC and RobotWebServices are active in the network that is being used.

:::tip

It is possible that this is not actually required for the simulation, but this step is definitely needed for the real robot.

:::

<div class="text--center">
  <img src={abbFirewallManager} alt="Firewall manager options." />
</div>

## RAPID module

While integrating in the AICA ecosystem, in the simplest case all that is required is a module that uses EGM to control
the robot externally. An example of such a module is as follows, users only need to make sure the UCDevice is the one
defined in the controller settings. The module can be placed in the controller's home directory, and uploaded to task by
right clicking on it and selecting Upload to Task. 

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
          TPWrite "Communication timedout";
          TRYNEXT;
      ENDIF
  ENDPROC
ENDMODULE
```

</details>


## Hardware interface

Returning to AICA studio and the hardware interface, it is now possible to define the parameters and connect to the
robot.The majority of the hardware interface parameters enable connection to EGM and RWS:

- EGM port: the port that EGM uses to send commands.
- RWS port & IP: port and address of the RWS.
- Connection timeout: for RWS connection.
- Rapid File Path: the path for the module to be loaded in the controller home directory.
- Target module name: the name for the module file in the controller home directory.
- Tf prefix: the prefix for the robot in the URDF file.

The EGM port should be set to the same value defined in the UDPUC device in the controller
configuration, and the RWS IP and port to the address of the Windows device and listening port respectively. Running the
application now can successfully connect to the simulated robot and get information about the setup. It also takes
advantage of RWS to automatically start and stop the simulation along with the application without any interaction with
RobotStudio. The robot is now ready to receive commands.

<div class="text--center">
  <img src={abbRSSuccessfulConnection} alt="Connected to RobotStudio successfully." />
</div>

