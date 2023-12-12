// @ts-check

const lightCodeTheme = require("prism-react-renderer/themes/github");
const darkCodeTheme = require("prism-react-renderer/themes/vsDark");

const url = "https://docs.aica.tech";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "AICA for Developers",
  tagline:
    "Documentation for the AICA Developer Interface and robotics software framework",
  favicon: "img/favicon.ico",

  url,
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
          sidebarPath: require.resolve("./src/layout/sidebars.js"),
          sidebarCollapsed: false,
          editUrl: "https://github.com/aica-technology/api/tree/main/docs",
        },
        blog: false,
        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      docs: {
        sidebar: {
          hideable: true,
        },
      },
      // TODO: add a social media card
      // image: "img/docusaurus-social-card.jpg",
      navbar: {
        title: "Documentation",
        logo: {
          alt: "AICA Logo",
          src: "img/logo.svg",
          srcDark: "img/logo_dark.svg",
        },
        items: [
          // TODO: add version dropdown when needed
          // {
          //   type: "docsVersionDropdown",
          //   position: "left",
          //   dropdownActiveClassDisabled: true,
          // },
          {
            type: "docSidebar",
            sidebarId: "conceptsSidebar",
            position: "left",
            label: "Concepts",
          },
          {
            type: "docSidebar",
            sidebarId: "gettingStartedSidebar",
            position: "left",
            label: "Getting started",
          },
          {
            type: "docSidebar",
            sidebarId: "programmingReferenceSidebar",
            position: "left",
            label: "Programming reference",
          },
          {
            href: `${url}/api`,
            label: "REST API",
            position: "left",
          },
          {
            href: "https://www.aica.tech",
            label: "Website",
            position: "right",
          },
          // TODO: need to apply to DocSearch https://docusaurus.io/docs/search
          {
            type: "search",
            position: "right",
          },
        ],
      },
      footer: {
        style: "dark",
        links: [
          {
            title: "More",
            items: [
              {
                label: "Website",
                href: "https://www.aica.tech",
              },
              {
                label: "GitHub",
                href: "https://github.com/aica-technology",
              },
              {
                label: "Contact",
                href: "mailto:contact@aica.tech",
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AICA SA`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        defaultLanguage: "python",
        additionalLanguages: ["toml"],
      },
    }),
};

module.exports = config;
