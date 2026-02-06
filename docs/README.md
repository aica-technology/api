# AICA Documentation

## [https://docs.aica.tech](https://docs.aica.tech)

The documentation is built using [Docusaurus 2](https://docusaurus.io/), a modern static website generator.

## Contributing

Contributions to the docs are welcome. Each page has an "Edit this page" link at the bottom which re-directs to the
corresponding source file in the GitHub repository. Make changes to the file on a branch or fork and open a Pull Request
to the `main` branch.

### Versioning

This project uses [multi-instance docs](https://docusaurus.io/docs/docs-multi-instance) for versioning. The main
unversioned docs instance lives in the [docs](./docs) directory, while a versioned instance is in the [core](./core)
directory.

The main docs should be considered evergreen and always up-to-date with the latest ecosystem concepts and overviews.
Even users of older version of the software should benefit from this kind of information.

The core docs are versioned with each major release of AICA Core, allowing specific guides and examples regarding the
usage of AICA Studio, the API and backend to be scoped appropriately.

Versioned pages show a banner and a version selection.

#### Maintaining docs with versioning

Older versions of core documentation are in the [core_versioned_docs](./core_versioned_docs) directory. Updating or
backporting any changes to older documentation can be done simply be editing the relevant files there. It may also be
necessary to update the appropriate [core_versioned_sidebars](./core_versioned_sidebars) in case of structural changes.

#### Cutting a new version

A new version of the core docs should be done only on major changes to AICA Core. To freeze the current state of the
core docs to a version (e.g. `v5`), use the following command:

```shell
bun run docusaurus docs:version:core v5
```

Then, in [docusaurus.config.js](./docusaurus.config.js) update the `versions` field of the
`@docusaurus/plugin-content-docs` plugin accordingly based on the following example:

Before:

```javascript
const config = {
    //...
    plugins: [
        [
            "@docusaurus/plugin-content-docs",
            {
                //...
                versions: {
                    current: {label: "AICA Core v5", path: "", banner: "none"},
                    v4: {label: "AICA Core v4", path: "v4", banner: "none"},
                }
            }
        ]
    ]
}
```

After:

```javascript
const config = {
    //...
    plugins: [
        [
            "@docusaurus/plugin-content-docs",
            {
                //...
                versions: {
                    current: {label: "AICA Core v6", path: "", banner: "none"},
                    v5: {label: "AICA Core v5", path: "v5", banner: "none"},
                    v4: {label: "AICA Core v4", path: "v4", banner: "none"},
                }
            }
        ]
    ]
}
```

### SVG assets with Excalidraw

Documentation diagrams should be made with [Excalidraw](https://excalidraw.com).

As a general style guide, use bold stroke width, "artist" stroke sloppiness, transparent backgrounds and hand-drawn
font with medium font-size.

To export a drawing, select the relevant parts of the scene and "Export image...". Under the export options, make
sure "Background" and "Dark mode" are **disabled**, and "Embed scene" is **enabled**. Then, export it as SVG.

Scene embedding allows collaborators to edit and extend existing diagrams. The exported SVG can be re-opened in
Excalidraw to continue editing it and then re-exported following the same steps.

To show the asset on a documentation page, import it and render
with [MDX syntax](https://docusaurus.io/docs/markdown-features/react). Use the custom `themedSVG` class to automatically
support light and dark themes. For example:

```markdown
import Example from './assets/exalidraw-example-embedded.svg';

<Example className="themedSVG" style={{width: "100%"}}/>
```

## Local development

### Installation

```
$ bun install
```

### Development

```
$ bun run start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without
having to restart the server.

### Build

```
$ bun run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting
service.

## Virtual development

You can build and see the documentation through Docker for ease of setup. Simply run:

```bash
docker compose up --build
```

You can then access the docs at [http://localhost:3000](http://localhost:3000).

## Deployment

The documentation is automatically built and deployed through a GitHub Actions workflow when changes are merged to
the `main` branch.

Sometimes, GitHub page deployment resets the custom domain setting, effectively breaking the documentation website.
To resolve this, go to the [API repository page settings](https://github.com/aica-technology/api/settings/pages) and,
under the **Custom domain** heading, set the custom domain to `docs.aica.tech`.
