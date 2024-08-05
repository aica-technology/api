import React from "react";
import clsx from "clsx";
import styles from "./styles.module.css";

type FeatureItem = {
    title: string;
    Svg?: React.ComponentType<React.ComponentProps<"svg">>;
    description: JSX.Element;
};

const FeatureList: FeatureItem[] = [
    {
        title: "Dynamic applications",
        description: (
            <>
                An AICA application is an interconnected graph of components, controllers and hardware interfaces.
                Components process data in a periodic step function and transfer data as signals to other components and
                controllers. Hardware interfaces are drivers that connect controllers to robots.
            </>
        ),
    },
    {
        title: "Built on ROS 2",
        description: (
            <>
                AICA embraces the ROS philosophies of modularity, distributed processing and open standards. For new
                developers, AICA provides an easy springboard into the wider world of ROS. Advanced users can take the
                modularity and dynamic nature of the AICA System to new frontiers.
            </>
        ),
    },
    {
        title: "Intelligent control",
        description: (
            <>
                AICA System is packed with cutting-edge technology for programming and controlling robots, including
                learning from demonstration, reinforcement learning and closed-loop force control. Start building
                dynamic, reactive, adaptive and collaborative control applications today.
            </>
        ),
    },
    {
        title: "AICA Studio",
        description: (
            <>
                An interactive application graph editor, URDF manager and API interface tools make creating, launching
                and monitoring AICA applications from your browser a breeze. Leverage the power of modular programming
                with a growing library of smart components and controllers in an intuitive drag and drop interface.
            </>
        ),
    },
];

function Feature({title, Svg, description}: FeatureItem) {
    return (
        <div className={clsx("col col--6")}>
            <div className="text--center">
                {Svg ? <Svg className={styles.featureSvg} role="img"/> : null}
            </div>
            <div className="padding-horiz--md">
                <h3>{title}</h3>
                <p>{description}</p>
            </div>
        </div>
    );
}

export default function HomepageFeatures(): JSX.Element {
    return (
        <section className={styles.features}>
            <div className="container">
                <div className="row">
                    {FeatureList.map((props, idx) => (
                        <Feature key={idx} {...props} />
                    ))}
                </div>
            </div>
        </section>
    );
}
