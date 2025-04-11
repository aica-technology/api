// @ts-check

const lightCodeTheme = require("prism-react-renderer").themes.github;
const darkCodeTheme = require("prism-react-renderer").themes.vsDark;

import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

const url = "https://docs.aica.tech";

/** @type {import('@docusaurus/types').Config} */
const config = {
    title: "AICA for Developers",
    tagline:
        "Documentation for the AICA System and robotics software framework",
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
                        href: "https://www.github.com/aica-technology",
                        label: "GitHub",
                        position: "right",
                    },
                    {
                        href: "https://aica.thinkific.com",
                        label: "AICAdemy",
                        position: "right",
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
                                label: "Discussions, announcements, and new releases",
                                href: "https://github.com/orgs/aica-technology/discussions",
                            },
                            {
                                label: "Report an issue",
                                href: "https://github.com/aica-technology/api/issues/new/choose",
                            },
                            {
                                label: "Website",
                                href: "https://www.aica.tech",
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
                additionalLanguages: ["cpp", "python", "toml", "json", "bash"],
            },
        }),
    stylesheets: [
        {
            href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
            type: 'text/css',
            integrity:
                'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
            crossorigin: 'anonymous',
        },
    ],
};

module.exports = config;
