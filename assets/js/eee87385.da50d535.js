"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[965],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>m});var r=n(7294);function o(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function a(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function i(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?a(Object(n),!0).forEach((function(t){o(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):a(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function c(e,t){if(null==e)return{};var n,r,o=function(e,t){if(null==e)return{};var n,r,o={},a=Object.keys(e);for(r=0;r<a.length;r++)n=a[r],t.indexOf(n)>=0||(o[n]=e[n]);return o}(e,t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(r=0;r<a.length;r++)n=a[r],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(o[n]=e[n])}return o}var l=r.createContext({}),s=function(e){var t=r.useContext(l),n=t;return e&&(n="function"==typeof e?e(t):i(i({},t),e)),n},p=function(e){var t=s(e.components);return r.createElement(l.Provider,{value:t},e.children)},d="mdxType",u={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},f=r.forwardRef((function(e,t){var n=e.components,o=e.mdxType,a=e.originalType,l=e.parentName,p=c(e,["components","mdxType","originalType","parentName"]),d=s(n),f=o,m=d["".concat(l,".").concat(f)]||d[f]||u[f]||a;return n?r.createElement(m,i(i({ref:t},p),{},{components:n})):r.createElement(m,i({ref:t},p))}));function m(e,t){var n=arguments,o=t&&t.mdxType;if("string"==typeof e||o){var a=n.length,i=new Array(a);i[0]=f;var c={};for(var l in t)hasOwnProperty.call(t,l)&&(c[l]=t[l]);c.originalType=e,c[d]="string"==typeof e?e:o,i[1]=c;for(var s=2;s<a;s++)i[s]=n[s];return r.createElement.apply(null,i)}return r.createElement.apply(null,n)}f.displayName="MDXCreateElement"},1519:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>l,contentTitle:()=>i,default:()=>u,frontMatter:()=>a,metadata:()=>c,toc:()=>s});var r=n(7462),o=(n(7294),n(3905));const a={sidebar_position:2},i="Lifecycle nodes",c={unversionedId:"concepts/ros-concepts/lifecycle",id:"concepts/ros-concepts/lifecycle",title:"Lifecycle nodes",description:'Lifecycle nodes, also referred to as "managed" nodes, extend the common interfaces of regular nodes with an internal',source:"@site/docs/concepts/03-ros-concepts/02-lifecycle.md",sourceDirName:"concepts/03-ros-concepts",slug:"/concepts/ros-concepts/lifecycle",permalink:"/api/docs/concepts/ros-concepts/lifecycle",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/concepts/03-ros-concepts/02-lifecycle.md",tags:[],version:"current",sidebarPosition:2,frontMatter:{sidebar_position:2},sidebar:"conceptsSidebar",previous:{title:"Nodes",permalink:"/api/docs/concepts/ros-concepts/nodes"},next:{title:"Dynamic composition",permalink:"/api/docs/concepts/ros-concepts/composition"}},l={},s=[],p={toc:s},d="wrapper";function u(e){let{components:t,...n}=e;return(0,o.kt)(d,(0,r.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,o.kt)("h1",{id:"lifecycle-nodes"},"Lifecycle nodes"),(0,o.kt)("p",null,'Lifecycle nodes, also referred to as "managed" nodes, extend the common interfaces of regular nodes with an internal\nstate machine following the concept of ',(0,o.kt)("em",{parentName:"p"},"lifecycle states"),"."),(0,o.kt)("p",null,"Nodes are normally launched as a collection of processes based on a launch configuration file. In complex applications,\nit may not be desirable for all node processes to start executing right away. For example, a computer vision processing\nnode should not perform any calculations until a camera sensor node is ready."),(0,o.kt)("p",null,"Lifecycle nodes present a solution to this problem by distinguishing between different activation states. Developers can\nthen implement different node behaviors depending on the current internal state."),(0,o.kt)("p",null,"A lifecycle node has four primary states:"),(0,o.kt)("ul",null,(0,o.kt)("li",{parentName:"ul"},"Unconfigured"),(0,o.kt)("li",{parentName:"ul"},"Inactive"),(0,o.kt)("li",{parentName:"ul"},"Active"),(0,o.kt)("li",{parentName:"ul"},"Finalized")),(0,o.kt)("p",null,"There are seven primary transitions between the states:"),(0,o.kt)("ul",null,(0,o.kt)("li",{parentName:"ul"},"Create"),(0,o.kt)("li",{parentName:"ul"},"Configure"),(0,o.kt)("li",{parentName:"ul"},"Activate"),(0,o.kt)("li",{parentName:"ul"},"Deactivate"),(0,o.kt)("li",{parentName:"ul"},"Cleanup"),(0,o.kt)("li",{parentName:"ul"},"Shutdown"),(0,o.kt)("li",{parentName:"ul"},"Destroy")),(0,o.kt)("p",null,"A lifecycle node is a managed node because the transitions between primary lifecycle states are normally triggered by an\nexternal process. In other words, a lifecycle node in the inactive state stays inactive until an external process (for\nexample, another node) requests a transition (for example, from inactive to active using the ",(0,o.kt)("inlineCode",{parentName:"p"},"activate")," transition)."),(0,o.kt)("p",null,"Another key benefit of lifecycle nodes is the ability to build sophisticated error handling and recovery behavior. An\nerror can be handled differently depending on the current state or transition. Instead of the node crashing and shutting\ndown completely, a properly handled error can recover the lifecycle node to an unconfigured state."),(0,o.kt)("admonition",{type:"info"},(0,o.kt)("p",{parentName:"admonition"},"Read more about managed nodes in ",(0,o.kt)("a",{parentName:"p",href:"https://design.ros2.org/articles/node_lifecycle.html"},"the ROS 2 design document"),".")),(0,o.kt)("p",null,"Using lifecycle nodes is one way to dynamically manage the behavior of an application at runtime. Another approach is\nto use dynamic composition, which is described on the next page."))}u.isMDXComponent=!0}}]);