import styles from "./index.module.css";
import React from "react";
import Layout from "@theme/Layout";

import aicaApi from "./assets/aica-api.png";
import aicaCommunity from "./assets/aica-community.png";
import aicademy from "./assets/aicademy.png";
import newRelease from "./assets/new-release.png";
import poweredAica from "./assets/powered-aica.png";

type ResourceItemProps = {
  title: string;
  description: string;
  href: string;
  image: string;
  alt: string;
};

function ResourceItem({ title, description, href, image, alt }: ResourceItemProps) {
  return (
    <div className={styles.item}>
      <div>
        <a href={href} target="_blank" rel="noopener noreferrer" className={styles.itemTitle}>
          {title}
        </a>

        <p className={styles.itemDescription}>{description}</p>
      </div>

      <img src={image} alt={alt} className={styles.itemImage} />
    </div>
  );
}

export default function ResourcesDocumentation() {
  return (
    <Layout>
      <div className="container margin-vert--lg">
        <h1 className="padding-bottom--md">Resources</h1>
        <p className="">
          Find helpful links to tutorials, community pages, and support options. Everything you need to go further with AICA.
        </p>
        
        <ResourceItem
          title="AICA API"
          description="REST API reference documentation."
          href="https://link.aica.tech/api"
          image={aicaApi}
          alt="AICA API preview"
        />

        <ResourceItem
          title="AICAdemy"
          description="Watch step-by-step free video tutorials to learn how to build applications, create custom components, and explore key feature at your own pace."
          href="https://link.aica.tech/aicademy"
          image={aicademy}
          alt="AICAdemy preview"
        />

        <ResourceItem
          title="AICA Community"
          description="Engage with AICA Community to discover planned features and improvements, submit your feedback, and propose your own ideas."
          href="https://link.aica.tech/community-gh"
          image={aicaCommunity}
          alt="AICA Community preview"
        />

        <ResourceItem
          title="Private Training & Support"
          description="Use support credits to access online or onsite support and training. Contact us: support@aica.tech"
          href="https://link.aica.tech/contact-support"
          image={poweredAica}
          alt="Powered by AICA Studio"
        />

        <ResourceItem
          title="Discussions & New Releases"
          description="A space for sharing announcements, discussing ideas, and staying updated on new releases."
          href="https://link.aica.tech/gh-discussions"
          image={newRelease}
          alt="New Release preview"
        />

      </div>
    </Layout>
  );
}