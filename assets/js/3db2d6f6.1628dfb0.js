"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[181],{3905:(e,t,a)=>{a.d(t,{Zo:()=>c,kt:()=>h});var n=a(7294);function r(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function o(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);t&&(n=n.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,n)}return a}function i(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?o(Object(a),!0).forEach((function(t){r(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):o(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function l(e,t){if(null==e)return{};var a,n,r=function(e,t){if(null==e)return{};var a,n,r={},o=Object.keys(e);for(n=0;n<o.length;n++)a=o[n],t.indexOf(a)>=0||(r[a]=e[a]);return r}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(n=0;n<o.length;n++)a=o[n],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(r[a]=e[a])}return r}var s=n.createContext({}),u=function(e){var t=n.useContext(s),a=t;return e&&(a="function"==typeof e?e(t):i(i({},t),e)),a},c=function(e){var t=u(e.components);return n.createElement(s.Provider,{value:t},e.children)},d="mdxType",p={inlineCode:"code",wrapper:function(e){var t=e.children;return n.createElement(n.Fragment,{},t)}},m=n.forwardRef((function(e,t){var a=e.components,r=e.mdxType,o=e.originalType,s=e.parentName,c=l(e,["components","mdxType","originalType","parentName"]),d=u(a),m=r,h=d["".concat(s,".").concat(m)]||d[m]||p[m]||o;return a?n.createElement(h,i(i({ref:t},c),{},{components:a})):n.createElement(h,i({ref:t},c))}));function h(e,t){var a=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var o=a.length,i=new Array(o);i[0]=m;var l={};for(var s in t)hasOwnProperty.call(t,s)&&(l[s]=t[s]);l.originalType=e,l[d]="string"==typeof e?e:r,i[1]=l;for(var u=2;u<o;u++)i[u]=a[u];return n.createElement.apply(null,i)}return n.createElement.apply(null,a)}m.displayName="MDXCreateElement"},5162:(e,t,a)=>{a.d(t,{Z:()=>i});var n=a(7294),r=a(6010);const o={tabItem:"tabItem_Ymn6"};function i(e){let{children:t,hidden:a,className:i}=e;return n.createElement("div",{role:"tabpanel",className:(0,r.Z)(o.tabItem,i),hidden:a},t)}},4866:(e,t,a)=>{a.d(t,{Z:()=>w});var n=a(7462),r=a(7294),o=a(6010),i=a(2466),l=a(6550),s=a(1980),u=a(7392),c=a(12);function d(e){return function(e){return r.Children.map(e,(e=>{if(!e||(0,r.isValidElement)(e)&&function(e){const{props:t}=e;return!!t&&"object"==typeof t&&"value"in t}(e))return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))?.filter(Boolean)??[]}(e).map((e=>{let{props:{value:t,label:a,attributes:n,default:r}}=e;return{value:t,label:a,attributes:n,default:r}}))}function p(e){const{values:t,children:a}=e;return(0,r.useMemo)((()=>{const e=t??d(a);return function(e){const t=(0,u.l)(e,((e,t)=>e.value===t.value));if(t.length>0)throw new Error(`Docusaurus error: Duplicate values "${t.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[t,a])}function m(e){let{value:t,tabValues:a}=e;return a.some((e=>e.value===t))}function h(e){let{queryString:t=!1,groupId:a}=e;const n=(0,l.k6)(),o=function(e){let{queryString:t=!1,groupId:a}=e;if("string"==typeof t)return t;if(!1===t)return null;if(!0===t&&!a)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return a??null}({queryString:t,groupId:a});return[(0,s._X)(o),(0,r.useCallback)((e=>{if(!o)return;const t=new URLSearchParams(n.location.search);t.set(o,e),n.replace({...n.location,search:t.toString()})}),[o,n])]}function f(e){const{defaultValue:t,queryString:a=!1,groupId:n}=e,o=p(e),[i,l]=(0,r.useState)((()=>function(e){let{defaultValue:t,tabValues:a}=e;if(0===a.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(t){if(!m({value:t,tabValues:a}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${t}" but none of its children has the corresponding value. Available values are: ${a.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return t}const n=a.find((e=>e.default))??a[0];if(!n)throw new Error("Unexpected error: 0 tabValues");return n.value}({defaultValue:t,tabValues:o}))),[s,u]=h({queryString:a,groupId:n}),[d,f]=function(e){let{groupId:t}=e;const a=function(e){return e?`docusaurus.tab.${e}`:null}(t),[n,o]=(0,c.Nk)(a);return[n,(0,r.useCallback)((e=>{a&&o.set(e)}),[a,o])]}({groupId:n}),g=(()=>{const e=s??d;return m({value:e,tabValues:o})?e:null})();(0,r.useLayoutEffect)((()=>{g&&l(g)}),[g]);return{selectedValue:i,selectValue:(0,r.useCallback)((e=>{if(!m({value:e,tabValues:o}))throw new Error(`Can't select invalid tab value=${e}`);l(e),u(e),f(e)}),[u,f,o]),tabValues:o}}var g=a(2389);const b={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};function v(e){let{className:t,block:a,selectedValue:l,selectValue:s,tabValues:u}=e;const c=[],{blockElementScrollPositionUntilNextRender:d}=(0,i.o5)(),p=e=>{const t=e.currentTarget,a=c.indexOf(t),n=u[a].value;n!==l&&(d(t),s(n))},m=e=>{let t=null;switch(e.key){case"Enter":p(e);break;case"ArrowRight":{const a=c.indexOf(e.currentTarget)+1;t=c[a]??c[0];break}case"ArrowLeft":{const a=c.indexOf(e.currentTarget)-1;t=c[a]??c[c.length-1];break}}t?.focus()};return r.createElement("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,o.Z)("tabs",{"tabs--block":a},t)},u.map((e=>{let{value:t,label:a,attributes:i}=e;return r.createElement("li",(0,n.Z)({role:"tab",tabIndex:l===t?0:-1,"aria-selected":l===t,key:t,ref:e=>c.push(e),onKeyDown:m,onClick:p},i,{className:(0,o.Z)("tabs__item",b.tabItem,i?.className,{"tabs__item--active":l===t})}),a??t)})))}function y(e){let{lazy:t,children:a,selectedValue:n}=e;const o=(Array.isArray(a)?a:[a]).filter(Boolean);if(t){const e=o.find((e=>e.props.value===n));return e?(0,r.cloneElement)(e,{className:"margin-top--md"}):null}return r.createElement("div",{className:"margin-top--md"},o.map(((e,t)=>(0,r.cloneElement)(e,{key:t,hidden:e.props.value!==n}))))}function k(e){const t=f(e);return r.createElement("div",{className:(0,o.Z)("tabs-container",b.tabList)},r.createElement(v,(0,n.Z)({},e,t)),r.createElement(y,(0,n.Z)({},e,t)))}function w(e){const t=(0,g.Z)();return r.createElement(k,(0,n.Z)({key:String(t)},e))}},4274:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>c,contentTitle:()=>s,default:()=>h,frontMatter:()=>l,metadata:()=>u,toc:()=>d});var n=a(7462),r=(a(7294),a(3905)),o=a(4866),i=a(5162);const l={sidebar_position:3},s="Running the image",u={unversionedId:"getting-started/run",id:"getting-started/run",title:"Running the image",description:"This guide assumes that the AICA runtime application image was tagged as aica-runtime. If the generated image has a",source:"@site/docs/getting-started/03-run.md",sourceDirName:"getting-started",slug:"/getting-started/run",permalink:"/api/docs/getting-started/run",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/getting-started/03-run.md",tags:[],version:"current",sidebarPosition:3,frontMatter:{sidebar_position:3},sidebar:"gettingStartedSidebar",previous:{title:"Installation",permalink:"/api/docs/getting-started/installation"},next:{title:"A basic application example",permalink:"/api/docs/getting-started/timer-example"}},c={},d=[{value:"Access the Developer Interface",id:"access-the-developer-interface",level:2},{value:"Access the REST API",id:"access-the-rest-api",level:2},{value:"Persistent user data",id:"persistent-user-data",level:2}],p={toc:d},m="wrapper";function h(e){let{components:t,...a}=e;return(0,r.kt)(m,(0,n.Z)({},p,a,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"running-the-image"},"Running the image"),(0,r.kt)("p",null,"This guide assumes that the AICA runtime application image was tagged as ",(0,r.kt)("inlineCode",{parentName:"p"},"aica-runtime"),". If the generated image has a\ndifferent name, change the name of in the following instructions accordingly."),(0,r.kt)("p",null,"You can start the AICA application container with the following command."),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"Change ",(0,r.kt)("inlineCode",{parentName:"p"},"/path/to/license")," in the command below to the location of the personal AICA developer license assigned to you.\nFor example, use ",(0,r.kt)("inlineCode",{parentName:"p"},"~/.aica-license.toml")," to keep the license file hidden in the home folder.")),(0,r.kt)(o.Z,{groupId:"os",mdxType:"Tabs"},(0,r.kt)(i.Z,{value:"linux",label:"Linux",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-bash"},"docker run -it --rm \\\n  --privileged \\\n  --net=host \\\n  -v /path/to/license:/license:ro \\\n  aica-runtime\n"))),(0,r.kt)(i.Z,{value:"mac",label:"macOS",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-bash"},"docker run -it --rm \\\n  --privileged \\\n  -p 3000:3000 -p 5000:5000 \\\n  -v /path/to/license:/license:ro \\\n  aica-runtime\n")))),(0,r.kt)("h2",{id:"access-the-developer-interface"},"Access the Developer Interface"),(0,r.kt)("p",null,"Visit ",(0,r.kt)("a",{parentName:"p",href:"http://localhost:3000"},"localhost:3000")," in the browser to view the Developer Interface."),(0,r.kt)("h2",{id:"access-the-rest-api"},"Access the REST API"),(0,r.kt)("p",null,"Visit ",(0,r.kt)("a",{parentName:"p",href:"http://localhost:5000"},"localhost:5000")," to see the Swagger homepage and documentation for the REST API."),(0,r.kt)("h2",{id:"persistent-user-data"},"Persistent user data"),(0,r.kt)("p",null,"AICA applications and URDF hardware can be uploaded to a user database through the API or Developer Interface UI.\nBecause the docker container is isolated from the host filesystem, the local database will be lost when the container\nexists. To persist local data between session, create a dedicated directory somewhere on the host. For example,\nuse ",(0,r.kt)("inlineCode",{parentName:"p"},"mkdir ~/.aica-data")," to keep the data folder hidden in the home folder. Then execute the normal run command with an\nadditional volume mount for the user data."),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"Change ",(0,r.kt)("inlineCode",{parentName:"p"},"/path/to/data")," in the command below to a desired location for the data (e.g., ",(0,r.kt)("inlineCode",{parentName:"p"},"~/.aica-data")," or elsewhere)")),(0,r.kt)(o.Z,{groupId:"os",mdxType:"Tabs"},(0,r.kt)(i.Z,{value:"linux",label:"Linux",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-bash"},"docker run -it --rm \\\n  --privileged \\\n  --net=host \\\n  -v /path/to/license:/license:ro \\\n  #highlight-next-line\n  -v /path/to/data:/data:rw \\\n  aica-runtime\n"))),(0,r.kt)(i.Z,{value:"mac",label:"macOS",mdxType:"TabItem"},(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-bash"},"docker run -it --rm \\\n  --privileged \\\n  -p 3000:3000 -p 5000:5000 \\\n  -v /path/to/license:/license:ro \\\n  #highlight-next-line\n  -v /path/to/data:/data:rw \\\n  aica-runtime\n")))))}h.isMDXComponent=!0}}]);