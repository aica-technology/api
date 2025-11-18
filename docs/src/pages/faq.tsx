import styles from "./index.module.css";
import Link from "@docusaurus/Link";
import Layout from "@theme/Layout";
import React from "react";
import { useState } from "react";

type FaqItem = {
	question: string;
	answer: JSX.Element;
};

const faqList: FaqItem[] = [
	{
		question: "What Operating Systems and hardware does AICA officially support?",
		answer: (
			<>
				AICA is constantly expanding its hardware and OS support. For the most current list, please consult the <Link to="../../docs/category/compatibility/">Compatibility page</Link> of docs.<br />If your system or hardware is not currently listed, we recommend that you open an issue on our <Link to="https://github.com/aica-technology/community">AICA Community</Link> page.<br />We are compatible with <Link to="../../docs/concepts/ros-concepts/controlling-robots/">ros2_control</Link>, an open-source framework for creating custom hardware drivers.
			</>
		),
	},
    {
		question: "How can I visualize my robot and its sensor data in AICA?",
		answer: (
			<>
                <p>
                    To visualize your robot in AICA, open the 3D view from the right panel or click the mini 3D view in the bottom-left corner to maximize it.
                </p>
                <p>
                    AICA also supports uploading custom URDF files. To do this, click on the Hardware tab in the top center of the screen and click then click Upload. Once uploaded, your URDF will be available in the hardware block. In order to visualize the robot in the 3D view, make sure the path to your mesh is correct and mounted to the AICA system docker container. 
                </p>
                <p>
                    To view sensor data, click on Live Data, then select the topic containing your force sensor measurements. AICA will automatically display a time-series plot of the sensor readings.
                </p>
			</>
		),
	},
    {
		question: "How do I access local files from my computer inside an AICA application?",
		answer: (
			<>
                <p>
                    The <b>AICA system</b> runs inside a <b>Docker container</b>, allowing users to mount various folders based on their needs.
                </p>
                <p>
                    To mount a folder, open <b>AICA Launcher</b>, go to your <b>Configuration</b> page, click on <b>Advanced Settings</b>, and then select <b>Add a volume mount (+)</b>. Specify the folder you want to mount and the path where it should appear inside the Docker container.               
                </p>
                <p>
                    Once the folder is mounted, you can access it within your AICA application by referencing its <b>local path</b> inside the container.
                </p>
			</>
		),
	},
    {
		question: "Can multiple developers work on the same AICA project?",
		answer: (
			<>
                While the AICA System does not currently support real-time, simultaneous collaboration (like Google Docs), you can easily share projects. We encourage you to directly export your application or configuration from within the AICA System. This file can then be shared with your team member (via email, GitHub, etc), who can then import it directly into the AICA System to view and edit it. 			
            </>
		),
	},
    {
		question: "How can I submit bug reports or feature requests to the AICA team?",
		answer: (
			<>
                The best place to submit bug reports or feature requests is on our public feedback tracker on the <Link to="https://github.com/aica-technology/community">AICA Community</Link> page. This allows our engineering team to track the issue, and other users can vote or add context. It is also visible when the work to address the feature request or bug is planned and implemented. 
			</>
		),
	},
    {
		question: "If I have a specific roadblock, how do I submit logs to the support team or otherwise receive support?",
		answer: (
			<>
                <p>
                    For specific, private roadblocks, we offer personalized support from AICA engineers. This is typically included with AICA System Licenses via support credits.                 
                </p>
                <p>
                    You can contact the support team directly at <Link to="mailto:support@aica.tech">support@aica.tech</Link>.               
                </p>
                <p>
                    To help us resolve your issue as quickly as possible, please include your application file, a description of the roadblock, and any relevant logs. If you have questions about your support credit balance, please contact your AICA sales representative.               
                </p>
			</>
		),
	},
    {
		question: "How do I get the latest version of AICA Launcher?",
		answer: (
			<>
                Find the latest version of AICA Launcher on <Link to="https://github.com/aica-technology/api/releases">https://github.com/aica-technology/api/releases</Link>
			</>
		),
	},
];

function Faq({ question, answer }: FaqItem) {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <div
      className={`${styles.faqCard} ${isOpen ? styles.open : ""}`}
      onClick={() => setIsOpen(!isOpen)}
    >
      <h3 className={styles.faqHeader}>
        {question}
        <span className={`${styles.faqToggle} ${isOpen ? styles.open : ""}`}>+</span>
      </h3>
      {isOpen && <div style={{ marginTop: "0.5rem", opacity: 0.95 }}>{answer}</div>}
    </div>
  );
}

export default function FAQ(): JSX.Element {
	return (
        <Layout>
            <div className="container margin-vert--lg">
                <h1 className="padding-bottom--md">Frequently Asked Questions</h1>
                {faqList.map((props, idx) => (
                    <Faq key={idx} {...props} />
                ))}
			</div>
        </Layout>	
    );
}
