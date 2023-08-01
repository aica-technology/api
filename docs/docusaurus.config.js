// @ts-check

const lightCodeTheme = require("prism-react-renderer/themes/github");
const darkCodeTheme = require("prism-react-renderer/themes/dracula");

const url = "https://docs.aica.tech";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "AICA Documentation",
  tagline: "Documentation for AICA Developer Interface",
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
        docs: false,
        // docs: {
        //   sidebarPath: require.resolve("./src/layout/sidebars.js"),
        //   editUrl:
        //     "https://github.com/aica-technology/api/tree/develop/docs/docs",
        // },
        // TODO: disabled the blog for now
        blog: false,
        // blog: {
        //   showReadingTime: true,
        //   editUrl:
        //     "https://github.com/aica-technology/api/tree/develop/docs/blog",
        // },
        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
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
          // TODO: add when needed
          // {
          //   type: "docsVersionDropdown",
          //   position: "left",
          //   dropdownActiveClassDisabled: true,
          // },
          // TODO: Example of how to add an entry to the docs
          // {
          //   type: "docSidebar",
          //   sidebarId: "tutorialSidebar",
          //   position: "left",
          //   label: "Tutorial",
          // },
          {
            href: `${url}/api`,
            label: "API",
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
          // TODO: Example of how to add an entry to the docs
          // {
          //   title: "Docs",
          //   items: [
          //     {
          //       label: "Tutorial",
          //       to: "/docs/intro",
          //     },
          //   ],
          // },
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
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AICA SA`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        defaultLanguage: "python",
      },
    }),
};

module.exports = config;
