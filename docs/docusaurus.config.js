// @ts-check

const lightCodeTheme = require("prism-react-renderer").themes.github;
const darkCodeTheme = require("prism-react-renderer").themes.vsDark;

import rehypeKatex from "rehype-katex";
import remarkMath from "remark-math";

const url = "https://docs.aica.tech";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "AICA for Developers",
  tagline: "Documentation for the AICA System and robotics software framework",
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
          current: { label: "AICA Core v5", path: "", banner: "none" },
          v4: { label: "AICA Core v4", path: "v4", banner: "none" },
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
      // TODO: add a social media card
      // image: "img/docusaurus-social-card.jpg",
      navbar: {
        title: "AICA Documentation",

        logo: {
          alt: "AICA Logo",
          src: "img/aica-logo-black-square-small.svg",
          srcDark: "img/aica-logo-white-square-small.svg",
          width: 22,
          height: 22,
          href: "docs/product/getting-started",
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
            label: "AICA Studio",
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
          alt: "AICA Logo",
          src: "img/logo.svg",
          srcDark: "img/logo_dark.svg",
          width: 42,
          height: 42,
        },
        links: [
          {
            title: "Product Knowledge Base",
            items: [
              {
                label: "Getting Started",
                href: "/docs/product/getting-started",
              },
              {
                label: "Concepts",
                href: "/docs/product/concepts",
              },
              {
                label: "Programming Reference",
                href: "/docs/product/reference",
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
            title: "AICA Studio",
            items: [
              {
                label: "Tour of AICA Studio",
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
                label: "AICA API",
                href: "https://link.aica.tech/api",
              },
              {
                label: "AICAdemy",
                href: "https://link.aica.tech/aicademy",
              },
              {
                label: "GitHub",
                href: "https://www.github.com/aica-technology",
              },
            ],
          },
          {
            title: "AICA",
            items: [
              {
                label: "AICA Community",
                href: "https://github.com/aica-technology/community",
              },
              {
                label: "Website",
                href: "https://www.aica.tech",
              },
              {
                label: "contact@aica.tech",
                href: "mailto:contact@aica.tech",
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
        copyright: `Copyright © ${new Date().getFullYear()} AICA SA`,
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
