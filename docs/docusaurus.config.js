// @ts-check

const lightCodeTheme = require("prism-react-renderer").themes.github;
const darkCodeTheme = require("prism-react-renderer").themes.vsDark;

import rehypeKatex from "rehype-katex";
import remarkMath from "remark-math";

const url = "https://docs.ai-can-change.tech";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Developer Docs",
  tagline: "Documentation for the Adaptive & Intelligent Control System and robotics software framework",
  favicon: "img/favicon.ico",

  url: url,
  baseUrl: "/",

  // GitHub pages deployment config.
  organizationName: "aica-technology",
  projectName: "api",

  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve("./src/layout/sidebars.ts"),
          sidebarCollapsed: false,
          editUrl: "https://github.com/aica-technology/api/tree/main/docs",
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
        },
        blog: false,
        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
      }),
    ],
  ],
  plugins: [
    [
      "@docusaurus/plugin-content-docs",
      {
        id: "core",
        path: "core",
        routeBasePath: "core",
        sidebarCollapsed: false,
        sidebarPath: require.resolve("./src/layout/coreSidebars.ts"),
        editUrl: "https://github.com/aica-technology/api/tree/main/core",
        remarkPlugins: [remarkMath],
        rehypePlugins: [rehypeKatex],
        lastVersion: "current",
        versions: {
          current: { label: "Core v5", path: "", banner: "none" },
          v4: { label: "Core v4", path: "v4", banner: "none" },
        },
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      docs: {
        sidebar: {
          hideable: false,
          autoCollapseCategories: false,
        },
      },
      announcementBar: {
        id: 'rebranding-announcement',
        content:
            'This organization and its content are not affiliated, associated, authorized, endorsed by, or in any way officially connected with aicas GmbH, Karlsruhe, Germany (www.aicas.com and https://github.com/aicas). Any references to “AICA” are strictly historic.',
        backgroundColor: '#fafbfc',
        textColor: '#091E42',
        isCloseable: true,
      },
      // TODO: add a social media card
      // image: "img/docusaurus-social-card.jpg",
      navbar: {
        title: "Documentation",

        logo: {
          alt: "Logo",
          src: "img/aica-logo-black-square-small.svg",
          srcDark: "img/aica-logo-white-square-small.svg",
          width: 22,
          height: 22,
          href: "docs/getting-started",
        },
        items: [
          // TODO: need to apply to DocSearch https://docusaurus.io/docs/search
          {
            type: "search",
            position: "left",
          },
          {
            label: "Knowledge Base",
            type: "docSidebar",
            sidebarId: "learnSidebar",
            position: "left",
          },
          {
            label: "Studio",
            type: "docSidebar",
            docsPluginId: "core",
            sidebarId: "studioSidebar",
            position: "left",
          },
          {
            type: "docsVersionDropdown",
            docsPluginId: "core",
            dropdownActiveClassDisabled: true,
            position: "right",
          },
          {
            label: "FAQ",
            href: `/faq`,
            position: "right",
          },
          {
            label: "Help",
            href: `/help`,
            position: "right",
          },
        ],
      },
      footer: {
        logo: {
          alt: "Logo",
          src: "img/aica-logo-black-square-small.svg",
          srcDark: "img/aica-logo-white-square-small.svg",
          width: 42,
          height: 42,
        },
        links: [
          {
            title: "Product Knowledge Base",
            items: [
              {
                label: "Getting Started",
                href: "/docs/getting-started",
              },
              {
                label: "Concepts",
                href: "/docs/concepts",
              },
              {
                label: "Programming Reference",
                href: "/docs/reference",
              },
              {
                label: "More Resources",
                href: "/help",
              },
              {
                label: "FAQ",
                href: "/faq",
              },
            ],
          },
          {
            title: "Studio",
            items: [
              {
                label: "Tour of Studio",
                href: "/core/studio/",
              },
              {
                label: "Examples",
                href: "/core/examples/",
              },
            ],
          },
          {
            title: "Resources",
            items: [
              {
                label: "API",
                href: "https://link.aica.tech/api",
              },
              {
                label: "AI Academy",
                href: "https://link.aica.tech/aicademy",
              },
              {
                label: "GitHub",
                href: "https://www.github.com/aica-technology",
              },
            ],
          },
          {
            title: "Company",
            items: [
              {
                label: "Community",
                href: "https://github.com/aica-technology/community",
              },
              {
                label: "Website",
                href: "https://www.ai-can-change.tech",
              },
              {
                label: "contact@ai-can-change.tech",
                href: "mailto:contact@ai-can-change.tech",
              },
              // {
              //     label: "Private Training & Support",
              //     href: "https://link.aica.tech/contact-support",
              // },
              // {
              //     label: "Discussions & New Releases",
              //     href: "https://link.aica.tech/gh-discussions",
              // },
              // {
              //     label: "Give Feedback",
              //     href: "https://link.aica.tech/report-issue",
              // },
            ],
          },
        ],
        // copyright: `Copyright © ${new Date().getFullYear()} AICA SA`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        defaultLanguage: "python",
        additionalLanguages: ["cpp", "python", "toml", "json", "bash"],
      },
    }),
  stylesheets: [
    {
      href: "https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css",
      type: "text/css",
      integrity: "sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM",
      crossorigin: "anonymous",
    },
  ],
};

module.exports = config;
