"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[326],{3905:(e,t,n)=>{n.d(t,{Zo:()=>p,kt:()=>g});var a=n(7294);function i(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function s(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){i(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function r(e,t){if(null==e)return{};var n,a,i=function(e,t){if(null==e)return{};var n,a,i={},o=Object.keys(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||(i[n]=e[n]);return i}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(i[n]=e[n])}return i}var l=a.createContext({}),c=function(e){var t=a.useContext(l),n=t;return e&&(n="function"==typeof e?e(t):s(s({},t),e)),n},p=function(e){var t=c(e.components);return a.createElement(l.Provider,{value:t},e.children)},m="mdxType",u={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},d=a.forwardRef((function(e,t){var n=e.components,i=e.mdxType,o=e.originalType,l=e.parentName,p=r(e,["components","mdxType","originalType","parentName"]),m=c(n),d=i,g=m["".concat(l,".").concat(d)]||m[d]||u[d]||o;return n?a.createElement(g,s(s({ref:t},p),{},{components:n})):a.createElement(g,s({ref:t},p))}));function g(e,t){var n=arguments,i=t&&t.mdxType;if("string"==typeof e||i){var o=n.length,s=new Array(o);s[0]=d;var r={};for(var l in t)hasOwnProperty.call(t,l)&&(r[l]=t[l]);r.originalType=e,r[m]="string"==typeof e?e:i,s[1]=r;for(var c=2;c<o;c++)s[c]=n[c];return a.createElement.apply(null,s)}return a.createElement.apply(null,n)}d.displayName="MDXCreateElement"},9796:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>l,contentTitle:()=>s,default:()=>u,frontMatter:()=>o,metadata:()=>r,toc:()=>c});var a=n(7462),i=(n(7294),n(3905));const o={sidebar_position:1},s="Signals",r={unversionedId:"concepts/building-blocks/signals",id:"concepts/building-blocks/signals",title:"Signals",description:"In the AICA framework, signals are designed to exchange continuous data between components and controllers in a running",source:"@site/docs/concepts/05-building-blocks/01-signals.md",sourceDirName:"concepts/05-building-blocks",slug:"/concepts/building-blocks/signals",permalink:"/api/docs/concepts/building-blocks/signals",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/concepts/05-building-blocks/01-signals.md",tags:[],version:"current",sidebarPosition:1,frontMatter:{sidebar_position:1},sidebar:"conceptsSidebar",previous:{title:"Application building blocks",permalink:"/api/docs/category/application-building-blocks"},next:{title:"Events",permalink:"/api/docs/concepts/building-blocks/events"}},l={},c=[{value:"Basic signal types",id:"basic-signal-types",level:2},{value:"State signals",id:"state-signals",level:2},{value:"Custom messages",id:"custom-messages",level:2}],p={toc:c},m="wrapper";function u(e){let{components:t,...n}=e;return(0,i.kt)(m,(0,a.Z)({},p,n,{components:t,mdxType:"MDXLayout"}),(0,i.kt)("h1",{id:"signals"},"Signals"),(0,i.kt)("p",null,"In the AICA framework, signals are designed to exchange continuous data between components and controllers in a running\napplication. They are an abstraction of ROS 2 topics that are assumed to exchange data at a regular, periodic frequency.\nThis makes them well-suited for use with signal processing components and controllers."),(0,i.kt)("p",null,'ROS 2 topics are messages sent from publishers to subscribers under a specific namespace (the message "topic")\nusing some predefined message format (the message "type"). As a result, topics enable ROS nodes to communicate in many\ndifferent network topologies and message formats.'),(0,i.kt)("p",null,"Often, the message topic and message type is hardcoded within the implementation of a ROS node. This can make it\ndifficult to rearrange the network topology of ROS nodes without modifying and recompiling the node implementation\nitself."),(0,i.kt)("p",null,"Signals then are ROS 2 publishers and subscribers with dynamically assigned topics and standardized message types.\nThis makes it easy to reconfigure the signal connections between different components and controllers in the application\ngraph without modifying or recompiling any source code. By using standard message types, the signal compatibility\nbetween components and controllers is also simplified."),(0,i.kt)("p",null,"Additionally, ROS 2 messages are data packets, not data objects. Parsing data from a message, manipulating it and\nwriting it back into a message can involve a fair amount of boilerplate code."),(0,i.kt)("p",null,"When developing an AICA component, signals are automatically converted to and from the corresponding data object."),(0,i.kt)("h2",{id:"basic-signal-types"},"Basic signal types"),(0,i.kt)("p",null,"The following standard message types are provided for signals."),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},"Boolean (true / false)"),(0,i.kt)("li",{parentName:"ul"},"Integer (whole numbers)"),(0,i.kt)("li",{parentName:"ul"},"Double (floating point numbers)"),(0,i.kt)("li",{parentName:"ul"},"Vector (array of floating point numbers)"),(0,i.kt)("li",{parentName:"ul"},"String (plain text)")),(0,i.kt)("h2",{id:"state-signals"},"State signals"),(0,i.kt)("p",null,"In robot control applications, the ",(0,i.kt)("em",{parentName:"p"},"state")," of a robot or other objects is highly important."),(0,i.kt)("p",null,"In robotics and control, one of the most useful data types is the state of an object, i.e. its spatial properties.\nThe joint angles of a robot arm, the 3D position and velocity of a flying drone, or the measurement of an accelerometer\nor force-torque sensor are all examples of instantaneous state variables."),(0,i.kt)("p",null,"AICA signals make it easy for components and controllers to exchange Cartesian and joint state variables in an\ninternally consistent way. For component developers, state signals are automatically converted into smart data classes\nthat provide useful functions for conversions, transformations and other manipulations."),(0,i.kt)("p",null,"The following state variables can be exchanged as signals:"),(0,i.kt)("ul",null,(0,i.kt)("li",{parentName:"ul"},"Joint state",(0,i.kt)("ul",{parentName:"li"},(0,i.kt)("li",{parentName:"ul"},"Positions"),(0,i.kt)("li",{parentName:"ul"},"Velocities"),(0,i.kt)("li",{parentName:"ul"},"Accelerations"),(0,i.kt)("li",{parentName:"ul"},"Torques"))),(0,i.kt)("li",{parentName:"ul"},"Cartesian state",(0,i.kt)("ul",{parentName:"li"},(0,i.kt)("li",{parentName:"ul"},"Pose",(0,i.kt)("ul",{parentName:"li"},(0,i.kt)("li",{parentName:"ul"},"Position"),(0,i.kt)("li",{parentName:"ul"},"Orientation"))),(0,i.kt)("li",{parentName:"ul"},"Twist",(0,i.kt)("ul",{parentName:"li"},(0,i.kt)("li",{parentName:"ul"},"Linear velocity"),(0,i.kt)("li",{parentName:"ul"},"Angular velocity"))),(0,i.kt)("li",{parentName:"ul"},"Acceleration",(0,i.kt)("ul",{parentName:"li"},(0,i.kt)("li",{parentName:"ul"},"Linear acceleration"),(0,i.kt)("li",{parentName:"ul"},"Angular acceleration"))),(0,i.kt)("li",{parentName:"ul"},"Wrench",(0,i.kt)("ul",{parentName:"li"},(0,i.kt)("li",{parentName:"ul"},"Force"),(0,i.kt)("li",{parentName:"ul"},"Torque")))))),(0,i.kt)("admonition",{type:"info"},(0,i.kt)("p",{parentName:"admonition"},"AICA state signals are built on the\nopen-source ",(0,i.kt)("a",{parentName:"p",href:"https://aica-technology.github.io/control-libraries/versions/v7.1.0/md__github_workspace_source_state_representation__r_e_a_d_m_e.html"},(0,i.kt)("inlineCode",{parentName:"a"},"state_representation")),"\nlibrary for Cartesian and joint state classes in C++ and Python.")),(0,i.kt)("h2",{id:"custom-messages"},"Custom messages"),(0,i.kt)("p",null,"The standard primitive and state message types are generally enough to cover the majority of messaging needs in\nan AICA application. Having a reduced message definition set is important to maximizing the modularity and compatibility\nof components. When components are connected by a signal in an application graph, the application interpreter will try\nto assert that the signals have a matching type."),(0,i.kt)("p",null,"However, any ROS 2 message can be implemented as a signal using the ",(0,i.kt)("inlineCode",{parentName:"p"},"custom")," signal type. As long as the custom type\nbetween two connected components has the same name, the application will be valid."),(0,i.kt)("admonition",{type:"tip"},(0,i.kt)("p",{parentName:"admonition"},"The AICA component library includes signal translator components for commonly used ROS messages (namely ",(0,i.kt)("inlineCode",{parentName:"p"},"std_msgs"),"\nand ",(0,i.kt)("inlineCode",{parentName:"p"},"geometry_msgs"),") for AICA components to communicate with traditional ROS nodes in an external process.")))}u.isMDXComponent=!0}}]);