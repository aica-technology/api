// @ts-check

const lightCodeTheme = require("prism-react-renderer").themes.github;
const darkCodeTheme = require("prism-react-renderer").themes.vsDark;

import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

const url = "https://docs.aica.tech/";

/** @type {import('@docusaurus/types').Config} */
const config = {
    title: "AICA for Developers",
    tagline:
        "Documentation for the AICA System and robotics software framework",
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
                    routeBasePath: "/",
                    sidebarPath: require.resolve("./src/layout/sidebars.js"),
                    sidebarCollapsed: true,
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
                title: "AICA Documentation",
                logo: {
                    alt: "AICA Logo",
                    src: "img/aica-logo-black-square-small.svg",
                    srcDark: "img/aica-logo-white-square-small.svg",
                    width: 22,
                    height: 22,
                    href: "/learn/Getting-started/intro",
                },
                items: [
                    // TODO: add version dropdown when needed
                    // {
                    //   type: "docsVersionDropdown",
                    //   position: "left",
                    //   dropdownActiveClassDisabled: true,
                    // },
                    // TODO: need to apply to DocSearch https://docusaurus.io/docs/search
                    {
                        type: "search",
                        position: "left",
                    },
                    {
                        to: "/",
                        type: "docSidebar",
                        sidebarId: "learnSidebar",
                        position: "left",
                        label: "Learn",
                    },
                ],
            },
            footer: {
                style: "dark",
                logo: {
                    alt: "AICA Logo",
                    src: "img/logo_dark.svg",
                    srcDark: "img/logo_dark.svg",
                    width: 42,
                    height: 42,
                },
                links: [
                        {
                        title: "Learn",
                        items: [
                            {
                                label: "Getting Started",
                                href: "/learn/Getting-started/intro"
                            },
                            {
                                label: "Concepts",
                                href: "/category/ros-2-concepts",
                            },
                            {
                                label: "Examples",
                                href: "/category/guides",
                            },
                            {
                                label: "Programming Reference",
                                href: "/learn/Reference/intro",
                            },
                        ],
                    },
                    {
                        title: "Resources",
                        items: [
                            {
                                label: "AICA API",
                                href: "https://link.aica.tech/api"
                            },
                            {
                                label: "AICAdemy",
                                href: "https://link.aica.tech/aicademy"
                            },
                        ],
                    },
                    {
                        title: "More",
                        items: [
                            {
                                label: "AICA Community",
                                href: "https://github.com/aica-technology/community"
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
            href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
            type: 'text/css',
            integrity:
                'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
            crossorigin: 'anonymous',
        },
    ],
};

module.exports = config;
