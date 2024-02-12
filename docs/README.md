# AICA Documentation

## [https://docs.aica.tech](https://docs.aica.tech)

The documentation is built using [Docusaurus 2](https://docusaurus.io/), a modern static website generator.

## Contributing

Contributions to the docs are welcome. Each page has an "Edit this page" link at the bottom which re-directs to the
corresponding source file in the GitHub repository. Make changes to the file on a branch or fork and open a Pull Request
to the `main` branch.

### SVG assets with Excalidraw

Documentation diagrams should be made with [Excalidraw](https://excalidraw.com).

As a general style guide, use bold stroke width, "artist" stroke sloppiness, transparent backgrounds and hand-drawn
font with medium font-size.

To exporting a drawing, select the relevant parts of the scene and "Export image...". Under the export options, make
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
$ npm install
```

### Development

```
$ npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without
having to restart the server.

### Build

```
$ npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting
service.

### Deployment

The documentation is automatically built and deployed through a GitHub Actions workflow when changes are merged to 
the `main` branch.

Sometimes, GitHub page deployment resets the custom domain setting, effectively breaking the documentation website.
To resolve this, go to the [API repository page settings](https://github.com/aica-technology/api/settings/pages) and,
under the **Custom domain** heading, set the custom domain to `docs.aica.tech`.
