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
				remarkPlugins: [remarkMath],
				rehypePlugins: [rehypeKatex],
				sidebarPath: "./src/layout/coreSidebars.ts",
				// TODO: define the versions and labels once v4 has been tagged
				// versions: {
				// 	current: { label: "AICA Core v5", path: "", banner: "none" },
				// 	v4: {label: "AICA Core v4", path: "v4"},
				// },
			},
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
					{
						type: "docSidebar",
						sidebarId: "gettingStartedSidebar",
						position: "left",
						label: "Getting started",
					},
					{
						type: "docSidebar",
						docsPluginId: "core",
						sidebarId: "studioSidebar",
						position: "left",
						label: "AICA Studio",
					},
					{
						type: "docSidebar",
						sidebarId: "conceptsSidebar",
						position: "left",
						label: "Concepts",
					},
					{
						type: "docSidebar",
						docsPluginId: "core",
						sidebarId: "examplesSidebar",
						position: "left",
						label: "Examples",
					},
					{
						type: "docSidebar",
						sidebarId: "programmingReferenceSidebar",
						position: "left",
						label: "Programming reference",
					},
					// TODO: add FAQ section
					// {
					//     type: "docSidebar",
					//     sidebarId: "faqSidebar",
					//     position: "left",
					//     label: "FAQ",
					// },
					{
						href: `${url}/api`,
						label: "REST API",
						position: "left",
					},
					// TODO: enable version dropdown once first version has been tagged
					// {
					// 	type: "docsVersionDropdown",
					// 	docsPluginId: "core",
					// 	position: "right",
					// 	dropdownActiveClassDisabled: true,
					// },
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
			href: "https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css",
			type: "text/css",
			integrity:
				"sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM",
			crossorigin: "anonymous",
		},
	],
};

module.exports = config;
