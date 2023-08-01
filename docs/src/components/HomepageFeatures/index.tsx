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
    title: "Universal Compatibility",
    description: (
      <>
        Our software solution is designed to seamlessly integrate with all kinds
        of robots. With AICA software, you no longer need to worry about
        programming each robot with a different software, as our platform is
        compatible with all of them.
      </>
    ),
  },
  {
    title: "Reinforcement Learning",
    description: (
      <>
        Our software solution leverages the latest in cutting-edge technology,
        including reinforcement learning and closed-loop force control. This
        allows for dynamic motion and precise control of your robotic arm,
        resulting in improved efficiency, productivity, and accuracy.
      </>
    ),
  },
  {
    title: "Intuitive Interface",
    description: (
      <>
        Our AICA application offers an intuitive and user-friendly interface
        that makes controlling and programming your robotic arm a breeze. With
        modular programming and learning from demonstration capabilities, our
        application streamlines the process of controlling your robot.
      </>
    ),
  },
  {
    title: "Tailored App",
    description: (
      <>
        At AICA, we understand that every business is unique, which is why we
        offer custom application development services to meet your specific
        requirements. If you need a specific application for a project, our team
        of experts is here to create a tailored solution just for you.
      </>
    ),
  },
];

function Feature({ title, Svg, description }: FeatureItem) {
  return (
    <div className={clsx("col col--6")}>
      <div className="text--center">
        {Svg ? <Svg className={styles.featureSvg} role="img" /> : null}
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
