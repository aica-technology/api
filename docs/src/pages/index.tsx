import React from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";

import styles from "./index.module.css";

type LinkButtonProps = {
    link: string;
    text: string
};

function LinkButton({link, text}: LinkButtonProps) {
    return <Link
        className="button button--outline button--secondary button--lg"
        to={link}
    >
        {text}
    </Link>
}

function HomepageHeader() {
    const {siteConfig} = useDocusaurusContext();
    return (
        <header className={clsx("hero hero--primary", styles.heroBanner)}>
            <div className="container">
                <h1 className="hero__title">{siteConfig.title}</h1>
                <p className="hero__subtitle">{siteConfig.tagline}</p>
                <div className={styles.centered}>
                    <LinkButton link={"/docs/getting-started/intro"} text={"Get up and running"}/>
                    <LinkButton link={"/docs/category/ros-2-concepts"} text={"Learn basic concepts"}/>
                    <LinkButton link={"/docs/category/guides"} text={"Guides and examples"}/>
                    <LinkButton link={"/docs/reference/intro"} text={"Advanced programming"}/>
                </div>
            </div>
        </header>
    );
}

function AICAdemySection() {
    return (
        <header className={clsx("hero", styles.heroBanner)}>
            <div className="container">
                <p className="hero__subtitle">
                    Visit our learning platform for more video lessons and training modules
                </p>
                <LinkButton link={"https://aica.thinkific.com"} text={"AICAdemy"}/>
            </div>
        </header>
    );
}

export default function Home(): JSX.Element {
    return (
        <Layout>
            <HomepageHeader/>
            <main>
                <HomepageFeatures/>
                <AICAdemySection/>
            </main>
        </Layout>
    );
}
