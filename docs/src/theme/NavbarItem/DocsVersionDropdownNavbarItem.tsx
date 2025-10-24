import type { WrapperProps } from "@docusaurus/types";
import useRouteContext from "@docusaurus/useRouteContext";
import type DocsVersionDropdownNavbarItemType from "@theme/NavbarItem/DocsVersionDropdownNavbarItem";
import DocsVersionDropdownNavbarItem from "@theme-original/NavbarItem/DocsVersionDropdownNavbarItem";
import type { ReactNode } from "react";

type Props = WrapperProps<typeof DocsVersionDropdownNavbarItemType>;

const VERSIONED_PLUGINS = ["core"];

// wrap the version dropdown to only show for the core versioned docs
export default function DocsVersionDropdownNavbarItemWrapper(
	props: Props,
): ReactNode {
	const { plugin, data } = useRouteContext();
	console.log(plugin, props);

	if (!VERSIONED_PLUGINS.includes(plugin.id)) {
		return;
	}

	return (
		<>
			<DocsVersionDropdownNavbarItem {...props} />
		</>
	);
}
