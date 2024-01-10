"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[774],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>m});var a=n(7294);function i(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function r(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function o(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?r(Object(n),!0).forEach((function(t){i(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):r(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,a,i=function(e,t){if(null==e)return{};var n,a,i={},r=Object.keys(e);for(a=0;a<r.length;a++)n=r[a],t.indexOf(n)>=0||(i[n]=e[n]);return i}(e,t);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);for(a=0;a<r.length;a++)n=r[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(i[n]=e[n])}return i}var s=a.createContext({}),c=function(e){var t=a.useContext(s),n=t;return e&&(n="function"==typeof e?e(t):o(o({},t),e)),n},p=function(e){var t=c(e.components);return a.createElement(s.Provider,{value:t},e.children)},g="mdxType",d={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},u=a.forwardRef((function(e,t){var n=e.components,i=e.mdxType,r=e.originalType,s=e.parentName,p=l(e,["components","mdxType","originalType","parentName"]),g=c(n),u=i,m=g["".concat(s,".").concat(u)]||g[u]||d[u]||r;return n?a.createElement(m,o(o({ref:t},p),{},{components:n})):a.createElement(m,o({ref:t},p))}));function m(e,t){var n=arguments,i=t&&t.mdxType;if("string"==typeof e||i){var r=n.length,o=new Array(r);o[0]=u;var l={};for(var s in t)hasOwnProperty.call(t,s)&&(l[s]=t[s]);l.originalType=e,l[g]="string"==typeof e?e:i,o[1]=l;for(var c=2;c<r;c++)o[c]=n[c];return a.createElement.apply(null,o)}return a.createElement.apply(null,n)}u.displayName="MDXCreateElement"},779:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>s,contentTitle:()=>o,default:()=>d,frontMatter:()=>r,metadata:()=>l,toc:()=>c});var a=n(7462),i=(n(7294),n(3905));const r={sidebar_position:3,title:"Installation"},o=void 0,l={unversionedId:"getting-started/installation",id:"getting-started/installation",title:"Installation",description:"Pre-requisites",source:"@site/docs/getting-started/03-installation.md",sourceDirName:"getting-started",slug:"/getting-started/installation",permalink:"/docs/getting-started/installation",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/getting-started/03-installation.md",tags:[],version:"current",sidebarPosition:3,frontMatter:{sidebar_position:3,title:"Installation"},sidebar:"gettingStartedSidebar",previous:{title:"Licensing",permalink:"/docs/getting-started/licensing"},next:{title:"Running the image",permalink:"/docs/getting-started/run"}},s={},c=[{value:"Pre-requisites",id:"pre-requisites",level:2},{value:"Logging in to the AICA package registry",id:"logging-in-to-the-aica-package-registry",level:2},{value:"Configuring AICA packages with a manifest file",id:"configuring-aica-packages-with-a-manifest-file",level:2},{value:"Configuring a minimal runtime image with the core base package",id:"configuring-a-minimal-runtime-image-with-the-core-base-package",level:3},{value:"Configuring a runtime image with add-on packages",id:"configuring-a-runtime-image-with-add-on-packages",level:3},{value:"Building an AICA runtime application image",id:"building-an-aica-runtime-application-image",level:2}],p={toc:c},g="wrapper";function d(e){let{components:t,...n}=e;return(0,i.kt)(g,(0,a.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,i.kt)("h2",{id:"pre-requisites"},"Pre-requisites"),(0,i.kt)("p",null,"AICA software is distributed with Docker images and executed with Docker containers."),(0,i.kt)("p",null,"Before proceeding, ",(0,i.kt)("a",{parentName:"p",href:"https://docs.docker.com/engine/install/"},"install Docker Engine")," on the host machine."),(0,i.kt)("admonition",{type:"info"},(0,i.kt)("p",{parentName:"admonition"},"For Ubuntu users, make sure to follow\nthe ",(0,i.kt)("a",{parentName:"p",href:"https://docs.docker.com/engine/install/linux-postinstall/"},"post installation steps")," to create the ",(0,i.kt)("inlineCode",{parentName:"p"},"docker")," group\nand add your user.")),(0,i.kt)("h2",{id:"logging-in-to-the-aica-package-registry"},"Logging in to the AICA package registry"),(0,i.kt)("p",null,"AICA software packages are hosted in a private container registry. Use a valid ",(0,i.kt)("a",{parentName:"p",href:"/docs/getting-started/licensing"},"license file")," to\nauthenticate docker to login and pull images from the registry using the following command:"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-shell"},"cat aica-license.toml | docker login registry.licensing.aica.tech -u USERNAME --password-stdin\n")),(0,i.kt)("admonition",{type:"note"},(0,i.kt)("p",{parentName:"admonition"},"The ",(0,i.kt)("inlineCode",{parentName:"p"},"USERNAME")," in the command can be left as-is and does not have to be replaced with any user-specific information.\nThis is because the authentication layer ignores the username and only uses the license key supplied as the password.")),(0,i.kt)("h2",{id:"configuring-aica-packages-with-a-manifest-file"},"Configuring AICA packages with a manifest file"),(0,i.kt)("p",null,"As seen in ",(0,i.kt)("a",{parentName:"p",href:"/docs/concepts/intro"},"Concepts: Introduction"),", AICA software comprises a collection of multiple packages\naround a unified framework. The base package includes the Dynamic State Engine, a core collection of components and\ncontrollers, the REST API and Developer Interface UI. Add-on packages include additional components or hardware\ncollections."),(0,i.kt)("p",null,"A runtime application image is configured using a simple ",(0,i.kt)("strong",{parentName:"p"},"manifest file")," defining the version of the base package\nto use and optionally defining additional add-on packages. The manifest file contains a custom docker syntax header\npointing to AICA's app-builder tool, and the ",(0,i.kt)("inlineCode",{parentName:"p"},"docker build")," command is used to bundle all listed packages into a final\nruntime image."),(0,i.kt)("h3",{id:"configuring-a-minimal-runtime-image-with-the-core-base-package"},"Configuring a minimal runtime image with the core base package"),(0,i.kt)("p",null,"The manifest file must contain a syntax header and a list of packages. The minimal version of the manifest includes\nonly the base package. The version of the base package can be changed according to the latest release."),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-toml",metastring:'title="aica-package.toml"',title:'"aica-package.toml"'},'#syntax=ghcr.io/aica-technology/app-builder:v1\n\n[packages]\n"@aica/base" = "v2.1.0"\n')),(0,i.kt)("h3",{id:"configuring-a-runtime-image-with-add-on-packages"},"Configuring a runtime image with add-on packages"),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-toml",metastring:'title="aica-package.toml"',title:'"aica-package.toml"'},'#syntax=ghcr.io/aica-technology/app-builder:v1\n\n[packages]\n"@aica/base" = "v2.1.0"\n\n# add components\n"@aica/components/signal-processing" = "v1.0.1"\n\n# add hardware collections \n"@aica/collections/ur-collection" = "v3.0.4"\n')),(0,i.kt)("p",null,"An AICA license includes specific entitlements that determine which add-on packages and versions can be accessed and\nused.\nAICA support will provide a package manifest with the latest included add-ons together with the respective license key."),(0,i.kt)("p",null,"For a full list of available add-ons for components and hardware collections, along with their latest versions and\nrelease notes, contact your AICA representative."),(0,i.kt)("h2",{id:"building-an-aica-runtime-application-image"},"Building an AICA runtime application image"),(0,i.kt)("p",null,"For a manifest file saved as ",(0,i.kt)("inlineCode",{parentName:"p"},"aica-package.toml"),", use the following command to build the runtime application."),(0,i.kt)("pre",null,(0,i.kt)("code",{parentName:"pre",className:"language-shell"},"docker build -f aica-package.toml -t aica-runtime .\n")),(0,i.kt)("p",null,"The command ",(0,i.kt)("inlineCode",{parentName:"p"},"docker image ls | grep aica-runtime")," should then list the ",(0,i.kt)("inlineCode",{parentName:"p"},"aica-runtime")," image."),(0,i.kt)("p",null,"Continue to the next section to learn how to start the application container."))}d.isMDXComponent=!0}}]);