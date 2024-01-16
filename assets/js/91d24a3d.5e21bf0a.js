"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[101],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>f});var a=n(7294);function i(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function r(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){i(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function l(e,t){if(null==e)return{};var n,a,i=function(e,t){if(null==e)return{};var n,a,i={},o=Object.keys(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||(i[n]=e[n]);return i}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(i[n]=e[n])}return i}var c=a.createContext({}),s=function(e){var t=a.useContext(c),n=t;return e&&(n="function"==typeof e?e(t):r(r({},t),e)),n},p=function(e){var t=s(e.components);return a.createElement(c.Provider,{value:t},e.children)},d="mdxType",u={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},m=a.forwardRef((function(e,t){var n=e.components,i=e.mdxType,o=e.originalType,c=e.parentName,p=l(e,["components","mdxType","originalType","parentName"]),d=s(n),m=i,f=d["".concat(c,".").concat(m)]||d[m]||u[m]||o;return n?a.createElement(f,r(r({ref:t},p),{},{components:n})):a.createElement(f,r({ref:t},p))}));function f(e,t){var n=arguments,i=t&&t.mdxType;if("string"==typeof e||i){var o=n.length,r=new Array(o);r[0]=m;var l={};for(var c in t)hasOwnProperty.call(t,c)&&(l[c]=t[c]);l.originalType=e,l[d]="string"==typeof e?e:i,r[1]=l;for(var s=2;s<o;s++)r[s]=n[s];return a.createElement.apply(null,r)}return a.createElement.apply(null,n)}m.displayName="MDXCreateElement"},516:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>c,contentTitle:()=>r,default:()=>u,frontMatter:()=>o,metadata:()=>l,toc:()=>s});var a=n(7462),i=(n(7294),n(3905));const o={sidebar_position:3},r="Components",l={unversionedId:"concepts/building-blocks/components",id:"concepts/building-blocks/components",title:"Components",description:"In the AICA application framework, components are the building blocks of advanced robot behaviors.",source:"@site/docs/concepts/05-building-blocks/03-components.md",sourceDirName:"concepts/05-building-blocks",slug:"/concepts/building-blocks/components",permalink:"/docs/concepts/building-blocks/components",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/concepts/05-building-blocks/03-components.md",tags:[],version:"current",sidebarPosition:3,frontMatter:{sidebar_position:3},sidebar:"conceptsSidebar",previous:{title:"Events",permalink:"/docs/concepts/building-blocks/events"},next:{title:"Controllers",permalink:"/docs/concepts/building-blocks/controllers"}},c={},s=[{value:"Periodic behavior",id:"periodic-behavior",level:2},{value:"Parameters",id:"parameters",level:2},{value:"Signals",id:"signals",level:2},{value:"Predicates",id:"predicates",level:2},{value:"Lifecycle predicates",id:"lifecycle-predicates",level:3},{value:"Auto lifecycle events",id:"auto-lifecycle-events",level:3},{value:"Services",id:"services",level:2}],p={toc:s},d="wrapper";function u(e){let{components:t,...n}=e;return(0,i.kt)(d,(0,a.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,i.kt)("h1",{id:"components"},"Components"),(0,i.kt)("p",null,"In the AICA application framework, components are the building blocks of advanced robot behaviors."),(0,i.kt)("p",null,"AICA components are wrappers for ",(0,i.kt)("a",{parentName:"p",href:"/docs/concepts/ros-concepts/nodes"},"ROS 2 nodes"),"\nand ",(0,i.kt)("a",{parentName:"p",href:"/docs/concepts/ros-concepts/lifecycle"},"lifecycle nodes"),"\nusing ",(0,i.kt)("a",{parentName:"p",href:"/docs/concepts/ros-concepts/composition"},"dynamic composition")," with additional abstractions for enhanced modularity\nand developer convenience."),(0,i.kt)("p",null,"Components process data in a periodic step function, are configured using parameters and transfer data as signals to\nother components and hardware interfaces. Key component states are published as predicates. Components may also expose\nspecific services."),(0,i.kt)("h2",{id:"periodic-behavior"},"Periodic behavior"),(0,i.kt)("p",null,"A key design concept for AICA components is the idea of periodic behavior."),(0,i.kt)("p",null,"In ROS, nodes are generally driven through callbacks, triggered either by a subscription, parameter change event,\nor service request. Nodes that wish to publish data continuously normally configure an internal timer to trigger a\ncallback function at regular intervals."),(0,i.kt)("p",null,"This periodic execution is built in to AICA components by default. Components have an execution rate defined by\na ",(0,i.kt)("inlineCode",{parentName:"p"},"rate")," parameter (the frequency in Hertz for the number of execution steps per second), and automatically publish data\nattributes and predicates periodically."),(0,i.kt)("p",null,"Components can implement specific calculations to occur on step."),(0,i.kt)("admonition",{type:"note"},(0,i.kt)("p",{parentName:"admonition"},"Lifecycle components only execute the periodic step function when they are in the ",(0,i.kt)("inlineCode",{parentName:"p"},"ACTIVE")," lifecycle state.")),(0,i.kt)("h2",{id:"parameters"},"Parameters"),(0,i.kt)("p",null,"Components support the standard ROS 2 parameter interface types."),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},"Boolean (true / false)"),(0,i.kt)("li",{parentName:"ul"},"Integer (whole numbers)"),(0,i.kt)("li",{parentName:"ul"},"Double (floating point numbers)"),(0,i.kt)("li",{parentName:"ul"},"String (plain text)")),(0,i.kt)("p",null,"They can additionally be an array (list) of multiple values of the same type:"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},"Boolean array"),(0,i.kt)("li",{parentName:"ul"},"Integer array"),(0,i.kt)("li",{parentName:"ul"},"Double array"),(0,i.kt)("li",{parentName:"ul"},"String array")),(0,i.kt)("h2",{id:"signals"},"Signals"),(0,i.kt)("p",null,"Components define signals as inputs (subscribers) and outputs (publishers). Refer to ",(0,i.kt)("a",{parentName:"p",href:"/docs/concepts/building-blocks/signals"},"Signals")," for more\ninformation."),(0,i.kt)("admonition",{type:"note"},(0,i.kt)("p",{parentName:"admonition"},"By default, lifecycle components only publish outputs when they are in the ",(0,i.kt)("inlineCode",{parentName:"p"},"ACTIVE")," lifecycle state.")),(0,i.kt)("h2",{id:"predicates"},"Predicates"),(0,i.kt)("p",null,"Components declare and broadcast key internal states as predicate messages, which are used by the Dynamic State Engine\nto trigger events. Refer to ",(0,i.kt)("a",{parentName:"p",href:"/docs/concepts/building-blocks/events"},"Events")," for more information."),(0,i.kt)("h3",{id:"lifecycle-predicates"},"Lifecycle predicates"),(0,i.kt)("p",null,"Lifecycle components are based on ",(0,i.kt)("a",{parentName:"p",href:"/docs/concepts/ros-concepts/lifecycle"},"lifecycle nodes")," and have additional predicates\nfor each lifecycle state:"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"is_unconfigured")),(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"is_inactive")),(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"is_active")),(0,i.kt)("li",{parentName:"ul"},(0,i.kt)("inlineCode",{parentName:"li"},"is_finalized"))),(0,i.kt)("h3",{id:"auto-lifecycle-events"},"Auto lifecycle events"),(0,i.kt)("p",null,"When a lifecycle components is loaded, it starts in the unconfigured state and normally requires an external trigger to\ntransition into different states."),(0,i.kt)("p",null,"In the AICA framework, lifecycle components can auto-configure and auto-activate themselves. Associating\nthe ",(0,i.kt)("inlineCode",{parentName:"p"},"is_unconfigured")," predicate with a ",(0,i.kt)("inlineCode",{parentName:"p"},"lifecycle: configure")," event enables the component to automatically configure\nitself. Equivalently, the ",(0,i.kt)("inlineCode",{parentName:"p"},"is_inactive")," predicate can be used to trigger a ",(0,i.kt)("inlineCode",{parentName:"p"},"lifecycle: activate")," event."),(0,i.kt)("h2",{id:"services"},"Services"),(0,i.kt)("p",null,"Components can provide service endpoints to trigger specific behaviors on demand. For compatibility with the application\nsyntax and the Dynamic State Engine, the component service are limited to one of two request types:"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},"Service request with no payload"),(0,i.kt)("li",{parentName:"ul"},"Service request with a string payload")),(0,i.kt)("p",null,"The service response object for both types contains an execution status and message and is automatically handled by the\nDynamic State Engine."))}u.isMDXComponent=!0}}]);