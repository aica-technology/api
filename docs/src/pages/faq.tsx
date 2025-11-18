import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import HomepageFeatures from "@site/src/components/HomepageFeatures";
import Layout from "@theme/Layout";
import clsx from "clsx";
import React from "react";

import styles from "./index.module.css";

export default function FAQ(): JSX.Element {
	return (
        <Layout>
            # Frequently Asked Questions
            
            What Operating Systems and hardware does AICA officially support? - Rachel
            AICA is constantly expanding its hardware and OS support. For the most current list, please consult the “Compatibility” page of docs. If your system or hardware is not currently listed, we recommend that you open an issue on our AICA Community page. We are compatible with ros2_control, an open-source framework for creating custom hardware drivers.
            
            How can I visualize my robot and its sensor data in AICA? - Yehya
            To visualize your robot in AICA, open the 3D view from the right panel or click the mini 3D view in the bottom-left corner to maximize it.
            AICA also supports uploading custom URDF files. To do this, click on the Hardware tab in the top center of the screen and click then click Upload. Once uploaded, your URDF will be available in the hardware block. In order to visualize the robot in the 3D view, make sure the path to your mesh is correct and mounted to the AICA system docker container.
            To view sensor data, click on Live Data, then select the topic containing your force sensor measurements. AICA will automatically display a time-series plot of the sensor readings.
            
            How do I access local files from my computer inside an AICA application? (Explain mount drive options) - Yehya
            The AICA system runs inside a Docker container, allowing users to mount various folders based on their needs.
            To mount a folder, open AICA Launcher, go to your Configuration page, click on Advanced Settings, and then select Add a volume mount (+). Specify the folder you want to mount and the path where it should appear inside the Docker container.
            Once the folder is mounted, you can access it within your AICA application by referencing its local path inside the container.
            
            Can multiple developers work on the same AICA project? - Rachel
            While the AICA System does not currently support real-time, simultaneous collaboration (like Google Docs), you can easily share projects. We encourage you to directly export your application or configuration from within the AICA System. This file can then be shared with your team member (via email, GitHub, etc), who can then import it directly into the AICA System to view and edit it.
            
            How can I submit bug reports or feature requests to the AICA team? - Rachel
            The best place to submit bug reports or feature requests is on our public feedback tracker on the AICA Community page. This allows our engineering team to track the issue, and other users can vote or add context. It is also visible when the work to address the feature request or bug is planned and implemented.
            
            If I have a specific roadblock, how do I submit logs to the support team or otherwise receive support? - Rachel
            For specific, private roadblocks, we offer personalized support from AICA engineers. This is typically included with AICA System Licenses via support credits.
            
            You can contact the support team directly at support@aica.tech.
            
            To help us resolve your issue as quickly as possible, please include your application file, a description of the roadblock, and any relevant logs. If you have questions about your support credit balance, please contact your AICA sales representative.
            
            How do I get the latest version of AICA Launcher?
            Find the latest version of AICA Launcher on https://github.com/aica-technology/api/releases
        </Layout>	
        );
}
