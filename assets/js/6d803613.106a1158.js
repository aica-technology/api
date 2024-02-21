"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[774],{3905:(e,a,t)=>{t.d(a,{Zo:()=>p,kt:()=>u});var n=t(7294);function i(e,a,t){return a in e?Object.defineProperty(e,a,{value:t,enumerable:!0,configurable:!0,writable:!0}):e[a]=t,e}function o(e,a){var t=Object.keys(e);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);a&&(n=n.filter((function(a){return Object.getOwnPropertyDescriptor(e,a).enumerable}))),t.push.apply(t,n)}return t}function r(e){for(var a=1;a<arguments.length;a++){var t=null!=arguments[a]?arguments[a]:{};a%2?o(Object(t),!0).forEach((function(a){i(e,a,t[a])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(t)):o(Object(t)).forEach((function(a){Object.defineProperty(e,a,Object.getOwnPropertyDescriptor(t,a))}))}return e}function c(e,a){if(null==e)return{};var t,n,i=function(e,a){if(null==e)return{};var t,n,i={},o=Object.keys(e);for(n=0;n<o.length;n++)t=o[n],a.indexOf(t)>=0||(i[t]=e[t]);return i}(e,a);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(n=0;n<o.length;n++)t=o[n],a.indexOf(t)>=0||Object.prototype.propertyIsEnumerable.call(e,t)&&(i[t]=e[t])}return i}var l=n.createContext({}),s=function(e){var a=n.useContext(l),t=a;return e&&(t="function"==typeof e?e(a):r(r({},a),e)),t},p=function(e){var a=s(e.components);return n.createElement(l.Provider,{value:a},e.children)},g="mdxType",d={inlineCode:"code",wrapper:function(e){var a=e.children;return n.createElement(n.Fragment,{},a)}},m=n.forwardRef((function(e,a){var t=e.components,i=e.mdxType,o=e.originalType,l=e.parentName,p=c(e,["components","mdxType","originalType","parentName"]),g=s(t),m=i,u=g["".concat(l,".").concat(m)]||g[m]||d[m]||o;return t?n.createElement(u,r(r({ref:a},p),{},{components:t})):n.createElement(u,r({ref:a},p))}));function u(e,a){var t=arguments,i=a&&a.mdxType;if("string"==typeof e||i){var o=t.length,r=new Array(o);r[0]=m;var c={};for(var l in a)hasOwnProperty.call(a,l)&&(c[l]=a[l]);c.originalType=e,c[g]="string"==typeof e?e:i,r[1]=c;for(var s=2;s<o;s++)r[s]=t[s];return n.createElement.apply(null,r)}return n.createElement.apply(null,t)}m.displayName="MDXCreateElement"},779:(e,a,t)=>{t.r(a),t.d(a,{assets:()=>l,contentTitle:()=>r,default:()=>d,frontMatter:()=>o,metadata:()=>c,toc:()=>s});var n=t(7462),i=(t(7294),t(3905));const o={sidebar_position:3,title:"Installation"},r=void 0,c={unversionedId:"getting-started/installation",id:"getting-started/installation",title:"Installation",description:"Pre-requisites",source:"@site/docs/getting-started/03-installation.md",sourceDirName:"getting-started",slug:"/getting-started/installation",permalink:"/docs/getting-started/installation",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/getting-started/03-installation.md",tags:[],version:"current",sidebarPosition:3,frontMatter:{sidebar_position:3,title:"Installation"},sidebar:"gettingStartedSidebar",previous:{title:"Licensing",permalink:"/docs/getting-started/licensing"},next:{title:"Running the image",permalink:"/docs/getting-started/run"}},l={},s=[{value:"Pre-requisites",id:"pre-requisites",level:2},{value:"The AICA package registry",id:"the-aica-package-registry",level:2},{value:"Listing available AICA packages",id:"listing-available-aica-packages",level:3},{value:"Logging in to the AICA package registry",id:"logging-in-to-the-aica-package-registry",level:3},{value:"Configuring AICA packages with a manifest file",id:"configuring-aica-packages-with-a-manifest-file",level:2},{value:"Configuring a minimal runtime image with the core base package",id:"configuring-a-minimal-runtime-image-with-the-core-base-package",level:3},{value:"Configuring a runtime image with add-on packages",id:"configuring-a-runtime-image-with-add-on-packages",level:3},{value:"Including custom packages",id:"including-custom-packages",level:3},{value:"Building an AICA runtime application image",id:"building-an-aica-runtime-application-image",level:2}],p={toc:s},g="wrapper";function d(e){let{components:a,...t}=e;return(0,i.kt)(g,(0,n.Z)({},p,t,{components:a,mdxType:"MDXLayout"}),(0,i.kt)("h2",{id:"pre-requisites"},"Pre-requisites"),(0,i.kt)("p",null,"AICA software is distributed with Docker images and executed with Docker containers."),(0,i.kt)("p",null,"Before proceeding, ",(0,i.kt)("a",{parentName:"p",href:"https://docs.docker.com/engine/install/"},"install Docker Engine")," on the host machine."),(0,i.kt)("admonition",{type:"info"},(0,i.kt)("p",{parentName:"admonition"},"For Ubuntu users, make sure to follow\nthe ",(0,i.kt)("a",{parentName:"p",href:"https://docs.docker.com/engine/install/linux-postinstall/"},"post installation steps")," to create the ",(0,i.kt)("inlineCode",{parentName:"p"},"docker")," group\nand add your user.")),(0,i.kt)("h2",{id:"the-aica-package-registry"},"The AICA package registry"),(0,i.kt)("p",null,"As seen in ",(0,i.kt)("a",{parentName:"p",href:"/docs/concepts/intro"},"Concepts: Introduction"),", AICA software comprises a collection of multiple packages\naround a unified framework. The base package includes the Dynamic State Engine, a core collection of components and\ncontrollers, the REST API and Developer Interface UI. Add-on packages include additional components or hardware\ncollections."),(0,i.kt)("p",null,"AICA software packages are hosted as docker images in a private container registry. Viewing and downloading packages\nfrom the registry requires a valid ",(0,i.kt)("a",{parentName:"p",href:"/docs/getting-started/licensing"},"license"),"."),(0,i.kt)("h3",{id:"listing-available-aica-packages"},"Listing available AICA packages"),(0,i.kt)("p",null,"To list available packages and versions, go to the official AICA registry page\nat ",(0,i.kt)("a",{parentName:"p",href:"https://registry.licensing.aica.tech"},"https://registry.licensing.aica.tech")," and enter your license key."),(0,i.kt)("admonition",{type:"tip"},(0,i.kt)("p",{parentName:"admonition"},"An AICA license includes specific entitlements that determine which add-on packages and versions can be accessed and\nused. To discover and access additional components and hardware collections, contact your AICA representative to upgrade\nyour license.")),(0,i.kt)("h3",{id:"logging-in-to-the-aica-package-registry"},"Logging in to the AICA package registry"),(0,i.kt)("p",null,"To authenticate docker to login and pull images from the registry, run the following command:"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-shell"},"cat aica-license.toml | docker login registry.licensing.aica.tech -u USERNAME --password-stdin\n")),(0,i.kt)("admonition",{type:"note"},(0,i.kt)("p",{parentName:"admonition"},"The ",(0,i.kt)("inlineCode",{parentName:"p"},"USERNAME")," in the command can be left as-is and does not have to be replaced with any user-specific information.\nThis is because the authentication layer ignores the username and only uses the license key supplied as the password.")),(0,i.kt)("h2",{id:"configuring-aica-packages-with-a-manifest-file"},"Configuring AICA packages with a manifest file"),(0,i.kt)("p",null,"A runtime application image is configured using a simple ",(0,i.kt)("strong",{parentName:"p"},"manifest file")," defining the version of the base package\nto use and optionally defining additional add-on packages. The manifest file contains a custom docker syntax header\npointing to AICA's app-builder tool, and the ",(0,i.kt)("inlineCode",{parentName:"p"},"docker build")," command is used to bundle all listed packages into a final\nruntime image."),(0,i.kt)("h3",{id:"configuring-a-minimal-runtime-image-with-the-core-base-package"},"Configuring a minimal runtime image with the core base package"),(0,i.kt)("p",null,"The manifest file must contain a syntax header and a list of packages. The minimal version of the manifest includes\nonly the base package. The version of the base package can be changed according to the latest release."),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-toml",metastring:'title="aica-package.toml"',title:'"aica-package.toml"'},'#syntax=ghcr.io/aica-technology/app-builder:v1\n\n[packages]\n"@aica/base" = "v3.0.0"\n')),(0,i.kt)("h3",{id:"configuring-a-runtime-image-with-add-on-packages"},"Configuring a runtime image with add-on packages"),(0,i.kt)("p",null,"A manifest can include additional components and hardware collections as add-on packages. For any available package\n",(0,i.kt)("a",{parentName:"p",href:"#listing-available-aica-packages"},"listed in the AICA registry"),", specify the package and version with the ",(0,i.kt)("inlineCode",{parentName:"p"},"@aica/"),"\nprefix. The following example manifest file includes two add-on packages: version 1.0.1 of the\n",(0,i.kt)("inlineCode",{parentName:"p"},"components/signal-processing")," component package and version 3.04 of the ",(0,i.kt)("inlineCode",{parentName:"p"},"collections/ur-collection")," hardware collection\npackage."),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-toml",metastring:'title="aica-package.toml"',title:'"aica-package.toml"'},'#syntax=ghcr.io/aica-technology/app-builder:v1\n\n[packages]\n"@aica/base" = "v3.0.0"\n\n# add components\n"@aica/components/signal-processing" = "v1.0.1"\n\n# add hardware collections \n"@aica/collections/ur-collection" = "v3.0.4"\n')),(0,i.kt)("h3",{id:"including-custom-packages"},"Including custom packages"),(0,i.kt)("p",null,"The AICA framework allows developers to build their\nown ",(0,i.kt)("a",{parentName:"p",href:"/docs/reference/custom-components/component-package"},"custom components"),". These packages can be included under\na custom name using the ",(0,i.kt)("inlineCode",{parentName:"p"},"docker-image://")," prefix to specify the docker image name or path. For example, a custom\ncomponent package that was locally built using ",(0,i.kt)("inlineCode",{parentName:"p"},"docker build [...] --tag my-custom-component-package")," could be included\nas ",(0,i.kt)("inlineCode",{parentName:"p"},"docker-image://my-custom-component-package"),". Community and third-party packages may also be available on other\ndocker registries such as DockerHub or GitHub Container Registry and can be included with the associated docker path."),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-toml",metastring:'title="aica-package.toml"',title:'"aica-package.toml"'},'#syntax=ghcr.io/aica-technology/app-builder:v1\n\n[packages]\n"@aica/base" = "v3.0.0"\n\n# add a custom package from a local docker image path\n"my-local-package" = "docker-image://my-custom-component-package"\n\n# add a package from any docker path such as GitHub Container Registry\n"my-ghcr-package" = "docker-image://ghcr.io/user/package:tag"\n')),(0,i.kt)("h2",{id:"building-an-aica-runtime-application-image"},"Building an AICA runtime application image"),(0,i.kt)("admonition",{type:"note"},(0,i.kt)("p",{parentName:"admonition"},(0,i.kt)("a",{parentName:"p",href:"#logging-in-to-the-aica-package-registry"},"Log in to the package registry")," before building the image to authorize docker\nto access AICA packages.")),(0,i.kt)("p",null,"Once the desired packages have been configured in a manifest file, a ",(0,i.kt)("inlineCode",{parentName:"p"},"docker build")," command can be used to build the\nruntime application image. In this example, a manifest file saved as ",(0,i.kt)("inlineCode",{parentName:"p"},"aica-package.toml")," is used to build an image\nwith the name ",(0,i.kt)("inlineCode",{parentName:"p"},"aica-runtime"),"."),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-shell"},"docker build -f aica-package.toml -t aica-runtime .\n")),(0,i.kt)("p",null,"The command ",(0,i.kt)("inlineCode",{parentName:"p"},"docker image ls | grep aica-runtime")," should then list the ",(0,i.kt)("inlineCode",{parentName:"p"},"aica-runtime")," image."),(0,i.kt)("p",null,"Continue to the next section to learn how to start the application container."))}d.isMDXComponent=!0}}]);