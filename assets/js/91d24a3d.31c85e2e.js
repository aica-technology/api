"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[101],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>b});var o=n(7294);function r(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function a(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);t&&(o=o.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,o)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?a(Object(n),!0).forEach((function(t){r(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):a(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function s(e,t){if(null==e)return{};var n,o,r=function(e,t){if(null==e)return{};var n,o,r={},a=Object.keys(e);for(o=0;o<a.length;o++)n=a[o],t.indexOf(n)>=0||(r[n]=e[n]);return r}(e,t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(o=0;o<a.length;o++)n=a[o],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(r[n]=e[n])}return r}var c=o.createContext({}),l=function(e){var t=o.useContext(c),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},p=function(e){var t=l(e.components);return o.createElement(c.Provider,{value:t},e.children)},u="mdxType",d={inlineCode:"code",wrapper:function(e){var t=e.children;return o.createElement(o.Fragment,{},t)}},m=o.forwardRef((function(e,t){var n=e.components,r=e.mdxType,a=e.originalType,c=e.parentName,p=s(e,["components","mdxType","originalType","parentName"]),u=l(n),m=r,b=u["".concat(c,".").concat(m)]||u[m]||d[m]||a;return n?o.createElement(b,i(i({ref:t},p),{},{components:n})):o.createElement(b,i({ref:t},p))}));function b(e,t){var n=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var a=n.length,i=new Array(a);i[0]=m;var s={};for(var c in t)hasOwnProperty.call(t,c)&&(s[c]=t[c]);s.originalType=e,s[u]="string"==typeof e?e:r,i[1]=s;for(var l=2;l<a;l++)i[l]=n[l];return o.createElement.apply(null,i)}return o.createElement.apply(null,n)}m.displayName="MDXCreateElement"},516:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>c,contentTitle:()=>i,default:()=>d,frontMatter:()=>a,metadata:()=>s,toc:()=>l});var o=n(7462),r=(n(7294),n(3905));const a={sidebar_position:3},i="Components",s={unversionedId:"concepts/building-blocks/components",id:"concepts/building-blocks/components",title:"Components",description:"In the AICA application framework, components are the building blocks of advanced robot behaviors.",source:"@site/docs/concepts/05-building-blocks/03-components.md",sourceDirName:"concepts/05-building-blocks",slug:"/concepts/building-blocks/components",permalink:"/docs/concepts/building-blocks/components",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/concepts/05-building-blocks/03-components.md",tags:[],version:"current",sidebarPosition:3,frontMatter:{sidebar_position:3},sidebar:"conceptsSidebar",previous:{title:"Events",permalink:"/docs/concepts/building-blocks/events"},next:{title:"Controllers",permalink:"/docs/concepts/building-blocks/controllers"}},c={},l=[{value:"Periodic behavior",id:"periodic-behavior",level:2},{value:"Parameters",id:"parameters",level:2},{value:"Signals",id:"signals",level:2},{value:"Predicates",id:"predicates",level:2},{value:"Services",id:"services",level:2}],p={toc:l},u="wrapper";function d(e){let{components:t,...n}=e;return(0,r.kt)(u,(0,o.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"components"},"Components"),(0,r.kt)("p",null,"In the AICA application framework, components are the building blocks of advanced robot behaviors."),(0,r.kt)("p",null,"AICA components are wrappers for ",(0,r.kt)("a",{parentName:"p",href:"/docs/concepts/ros-concepts/nodes"},"ROS 2 nodes"),"\nand ",(0,r.kt)("a",{parentName:"p",href:"/docs/concepts/ros-concepts/lifecycle"},"lifecycle nodes"),"\nusing ",(0,r.kt)("a",{parentName:"p",href:"/docs/concepts/ros-concepts/composition"},"dynamic composition")," with additional abstractions for enhanced modularity\nand developer convenience."),(0,r.kt)("p",null,"Components process data in a periodic step function, are configured using parameters and transfer data as signals to\nother components and hardware interfaces. Key component states are published as predicates. Components may also expose\nspecific services."),(0,r.kt)("h2",{id:"periodic-behavior"},"Periodic behavior"),(0,r.kt)("p",null,"A key design concept for AICA components is the idea of periodic behavior."),(0,r.kt)("p",null,"In ROS, nodes are generally driven through callbacks, triggered either by a subscription, parameter change event,\nor service request. Nodes that wish to publish data continuously normally configure an internal timer to trigger a\ncallback function at regular intervals."),(0,r.kt)("p",null,"This periodic execution is built in to AICA components by default. Components have an execution rate defined by\na ",(0,r.kt)("inlineCode",{parentName:"p"},"rate")," parameter (the frequency in Hertz for the number of execution steps per second), and automatically publish data\nattributes and predicates periodically."),(0,r.kt)("p",null,"Components can implement specific calculations to occur on step."),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"Lifecycle components only execute the periodic step function when they are in the ",(0,r.kt)("inlineCode",{parentName:"p"},"ACTIVE")," lifecycle state.")),(0,r.kt)("h2",{id:"parameters"},"Parameters"),(0,r.kt)("p",null,"Components support the standard ROS 2 parameter interface types."),(0,r.kt)("ul",null,(0,r.kt)("li",{parentName:"ul"},"Boolean (true / false)"),(0,r.kt)("li",{parentName:"ul"},"Integer (whole numbers)"),(0,r.kt)("li",{parentName:"ul"},"Double (floating point numbers)"),(0,r.kt)("li",{parentName:"ul"},"String (plain text)")),(0,r.kt)("p",null,"They can additionally be an array (list) of multiple values of the same type:"),(0,r.kt)("ul",null,(0,r.kt)("li",{parentName:"ul"},"Boolean array"),(0,r.kt)("li",{parentName:"ul"},"Integer array"),(0,r.kt)("li",{parentName:"ul"},"Double array"),(0,r.kt)("li",{parentName:"ul"},"String array")),(0,r.kt)("h2",{id:"signals"},"Signals"),(0,r.kt)("p",null,"Components define signals as inputs (subscribers) and outputs (publishers). Refer to ",(0,r.kt)("a",{parentName:"p",href:"/docs/concepts/building-blocks/signals"},"Signals")," for more\ninformation."),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"By default, lifecycle components only publish outputs when they are in the ",(0,r.kt)("inlineCode",{parentName:"p"},"ACTIVE")," lifecycle state.")),(0,r.kt)("h2",{id:"predicates"},"Predicates"),(0,r.kt)("p",null,"Components declare and broadcast key internal states as predicate messages, which are used by the Dynamic State Engine\nto trigger events. Refer to ",(0,r.kt)("a",{parentName:"p",href:"/docs/concepts/building-blocks/events"},"Events")," for more information."),(0,r.kt)("h2",{id:"services"},"Services"),(0,r.kt)("p",null,"Components can provide service endpoints to trigger specific behaviors on demand. For compatibility with the application\nsyntax and the Dynamic State Engine, the component service are limited to one of two request types:"),(0,r.kt)("ul",null,(0,r.kt)("li",{parentName:"ul"},"Service request with no payload"),(0,r.kt)("li",{parentName:"ul"},"Service request with a string payload")),(0,r.kt)("p",null,"The service response object for both types contains an execution status and message and is automatically handled by the\nDynamic State Engine."))}d.isMDXComponent=!0}}]);