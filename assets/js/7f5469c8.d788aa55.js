"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[201],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>g});var a=n(7294);function r(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function i(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function o(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?i(Object(n),!0).forEach((function(t){r(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):i(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function c(e,t){if(null==e)return{};var n,a,r=function(e,t){if(null==e)return{};var n,a,r={},i=Object.keys(e);for(a=0;a<i.length;a++)n=i[a],t.indexOf(n)>=0||(r[n]=e[n]);return r}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(a=0;a<i.length;a++)n=i[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(r[n]=e[n])}return r}var l=a.createContext({}),s=function(e){var t=a.useContext(l),n=t;return e&&(n="function"==typeof e?e(t):o(o({},t),e)),n},p=function(e){var t=s(e.components);return a.createElement(l.Provider,{value:t},e.children)},d="mdxType",u={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},m=a.forwardRef((function(e,t){var n=e.components,r=e.mdxType,i=e.originalType,l=e.parentName,p=c(e,["components","mdxType","originalType","parentName"]),d=s(n),m=r,g=d["".concat(l,".").concat(m)]||d[m]||u[m]||i;return n?a.createElement(g,o(o({ref:t},p),{},{components:n})):a.createElement(g,o({ref:t},p))}));function g(e,t){var n=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var i=n.length,o=new Array(i);o[0]=m;var c={};for(var l in t)hasOwnProperty.call(t,l)&&(c[l]=t[l]);c.originalType=e,c[d]="string"==typeof e?e:r,o[1]=c;for(var s=2;s<i;s++)o[s]=n[s];return a.createElement.apply(null,o)}return a.createElement.apply(null,n)}m.displayName="MDXCreateElement"},5681:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>l,contentTitle:()=>o,default:()=>u,frontMatter:()=>i,metadata:()=>c,toc:()=>s});var a=n(7462),r=(n(7294),n(3905));const i={sidebar_position:2},o="Events",c={unversionedId:"concepts/building-blocks/events",id:"concepts/building-blocks/events",title:"Events",description:"Events are discrete actions that change the dynamic state of an AICA application. They are handled and executed by the",source:"@site/docs/concepts/05-building-blocks/02-events.md",sourceDirName:"concepts/05-building-blocks",slug:"/concepts/building-blocks/events",permalink:"/docs/concepts/building-blocks/events",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/concepts/05-building-blocks/02-events.md",tags:[],version:"current",sidebarPosition:2,frontMatter:{sidebar_position:2},sidebar:"conceptsSidebar",previous:{title:"Signals",permalink:"/docs/concepts/building-blocks/signals"},next:{title:"Components",permalink:"/docs/concepts/building-blocks/components"}},l={},s=[{value:"Event types",id:"event-types",level:2},{value:"Triggering events",id:"triggering-events",level:2},{value:"Predicates",id:"predicates",level:3},{value:"Conditions",id:"conditions",level:3}],p={toc:s},d="wrapper";function u(e){let{components:t,...n}=e;return(0,r.kt)(d,(0,a.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"events"},"Events"),(0,r.kt)("p",null,'Events are discrete actions that change the dynamic state of an AICA application. They are handled and executed by the\nDynamic State Engine. For this reason, they are sometimes referred to as "state events".'),(0,r.kt)("h2",{id:"event-types"},"Event types"),(0,r.kt)("p",null,"The following events can be used to dynamically update the state of a running application:"),(0,r.kt)("ul",null,(0,r.kt)("li",{parentName:"ul"},"Load or unload a component, controller or hardware interface"),(0,r.kt)("li",{parentName:"ul"},"Trigger a lifecycle transition (for example, configure, activate or deactivate) on a lifecycle component"),(0,r.kt)("li",{parentName:"ul"},"Start or stop a controller"),(0,r.kt)("li",{parentName:"ul"},"Set the value of a parameter on a loaded component or controller"),(0,r.kt)("li",{parentName:"ul"},"Call a service on a component")),(0,r.kt)("h2",{id:"triggering-events"},"Triggering events"),(0,r.kt)("p",null,"Events can be triggered externally by a user through interactions with the Developer Interface UI or via the REST API.\nThey can also be triggered internally by the Dynamic State Engine according to the application description as a result\nof component ",(0,r.kt)("strong",{parentName:"p"},"predicates")," and ",(0,r.kt)("strong",{parentName:"p"},"conditions"),"."),(0,r.kt)("h3",{id:"predicates"},"Predicates"),(0,r.kt)("p",null,"Predicates are logical statements about a component that evaluate to true or false and are used to indicate key\ncomponent states. AICA components broadcast their predicates to a global channel in a predicate message containing the\ncomponent name, the predicate name, and the current value (true or false) of the predicate."),(0,r.kt)("p",null,'While the term "predicate" has several formal definitions in grammar, logic and mathematics, at AICA the ',(0,r.kt)("a",{parentName:"p",href:"https://en.wikipedia.org/wiki/Predicate_(grammar)"},"grammatical\ndefinition")," is used when naming predicates."),(0,r.kt)("p",null,"For example, lifecycle components have a predicate for each lifecycle state:"),(0,r.kt)("ul",null,(0,r.kt)("li",{parentName:"ul"},"is unconfigured"),(0,r.kt)("li",{parentName:"ul"},"is inactive"),(0,r.kt)("li",{parentName:"ul"},"is active"),(0,r.kt)("li",{parentName:"ul"},"is finalized")),(0,r.kt)("p",null,'Specialized components define additional predicates depending on their function. A component that calculates if a given\ninput state is within some parameterized limits might have a predicate "is in bounds".'),(0,r.kt)("p",null,"This definition of predicates allows application descriptions to trigger events in a very declarative way. For example:"),(0,r.kt)("blockquote",null,(0,r.kt)("p",{parentName:"blockquote"},"When component A ",(0,r.kt)("em",{parentName:"p"},"is active"),", load component B")),(0,r.kt)("p",null,"Here, ",(0,r.kt)("inlineCode",{parentName:"p"},"component A")," is the source of the predicate, ",(0,r.kt)("inlineCode",{parentName:"p"},"is active")," is the predicate itself, and ",(0,r.kt)("inlineCode",{parentName:"p"},"load component B")," is the\nevent that is triggered when the predicate is true."),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"Events are only triggered on the ",(0,r.kt)("a",{parentName:"p",href:"https://en.wikipedia.org/wiki/Signal_edge"},(0,r.kt)("em",{parentName:"a"},"rising edge"))," of a predicate value.")),(0,r.kt)("h3",{id:"conditions"},"Conditions"),(0,r.kt)("p",null,"Predicate statements can be manipulated with logical operators in conditional statements to create more advanced event\ntriggers."),(0,r.kt)("blockquote",null,(0,r.kt)("p",{parentName:"blockquote"},"When component B ",(0,r.kt)("em",{parentName:"p"},"is ",(0,r.kt)("strong",{parentName:"em"},"not")," in bounds"),", do ...")),(0,r.kt)("blockquote",null,(0,r.kt)("p",{parentName:"blockquote"},"When component A ",(0,r.kt)("em",{parentName:"p"},"is active")," ",(0,r.kt)("strong",{parentName:"p"},"and")," component B ",(0,r.kt)("em",{parentName:"p"},"is ",(0,r.kt)("strong",{parentName:"em"},"not")," in bounds"),", do ...")),(0,r.kt)("p",null,'AICA conditions support the NOT, AND, OR and XOR operators (also known as the "not", "all", "any", and "one of"\noperators, respectively).'),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"Events are only triggered on the rising edge of a conditional value.")))}u.isMDXComponent=!0}}]);