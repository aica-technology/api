"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[839],{3905:(e,t,o)=>{o.d(t,{Zo:()=>p,kt:()=>f});var r=o(7294);function n(e,t,o){return t in e?Object.defineProperty(e,t,{value:o,enumerable:!0,configurable:!0,writable:!0}):e[t]=o,e}function a(e,t){var o=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),o.push.apply(o,r)}return o}function i(e){for(var t=1;t<arguments.length;t++){var o=null!=arguments[t]?arguments[t]:{};t%2?a(Object(o),!0).forEach((function(t){n(e,t,o[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(o)):a(Object(o)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(o,t))}))}return e}function s(e,t){if(null==e)return{};var o,r,n=function(e,t){if(null==e)return{};var o,r,n={},a=Object.keys(e);for(r=0;r<a.length;r++)o=a[r],t.indexOf(o)>=0||(n[o]=e[o]);return n}(e,t);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);for(r=0;r<a.length;r++)o=a[r],t.indexOf(o)>=0||Object.prototype.propertyIsEnumerable.call(e,o)&&(n[o]=e[o])}return n}var c=r.createContext({}),l=function(e){var t=r.useContext(c),o=t;return e&&(o="function"==typeof e?e(t):i(i({},t),e)),o},p=function(e){var t=l(e.components);return r.createElement(c.Provider,{value:t},e.children)},d="mdxType",m={inlineCode:"code",wrapper:function(e){var t=e.children;return r.createElement(r.Fragment,{},t)}},u=r.forwardRef((function(e,t){var o=e.components,n=e.mdxType,a=e.originalType,c=e.parentName,p=s(e,["components","mdxType","originalType","parentName"]),d=l(o),u=n,f=d["".concat(c,".").concat(u)]||d[u]||m[u]||a;return o?r.createElement(f,i(i({ref:t},p),{},{components:o})):r.createElement(f,i({ref:t},p))}));function f(e,t){var o=arguments,n=t&&t.mdxType;if("string"==typeof e||n){var a=o.length,i=new Array(a);i[0]=u;var s={};for(var c in t)hasOwnProperty.call(t,c)&&(s[c]=t[c]);s.originalType=e,s[d]="string"==typeof e?e:n,i[1]=s;for(var l=2;l<a;l++)i[l]=o[l];return r.createElement.apply(null,i)}return r.createElement.apply(null,o)}u.displayName="MDXCreateElement"},5370:(e,t,o)=>{o.r(t),o.d(t,{assets:()=>c,contentTitle:()=>i,default:()=>m,frontMatter:()=>a,metadata:()=>s,toc:()=>l});var r=o(7462),n=(o(7294),o(3905));const a={sidebar_position:4,title:"Controlling robots"},i="Controlling robots with ros2_control",s={unversionedId:"concepts/ros-concepts/control",id:"concepts/ros-concepts/control",title:"Controlling robots",description:"ROS nodes are easy to conceptualize as pure functional components that process input topics and generate output",source:"@site/docs/concepts/03-ros-concepts/04-control.md",sourceDirName:"concepts/03-ros-concepts",slug:"/concepts/ros-concepts/control",permalink:"/docs/concepts/ros-concepts/control",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/concepts/03-ros-concepts/04-control.md",tags:[],version:"current",sidebarPosition:4,frontMatter:{sidebar_position:4,title:"Controlling robots"},sidebar:"conceptsSidebar",previous:{title:"Dynamic composition",permalink:"/docs/concepts/ros-concepts/composition"},next:{title:"AICA applications",permalink:"/docs/concepts/aica-applications"}},c={},l=[{value:"Controller Manager",id:"controller-manager",level:2},{value:"State interfaces",id:"state-interfaces",level:3},{value:"Command interfaces",id:"command-interfaces",level:3},{value:"Hardware interfaces",id:"hardware-interfaces",level:2},{value:"Controllers",id:"controllers",level:2}],p={toc:l},d="wrapper";function m(e){let{components:t,...o}=e;return(0,n.kt)(d,(0,r.Z)({},p,o,{components:t,mdxType:"MDXLayout"}),(0,n.kt)("h1",{id:"controlling-robots-with-ros2_control"},"Controlling robots with ros2_control"),(0,n.kt)("p",null,"ROS nodes are easy to conceptualize as pure functional components that process input topics and generate output\ntopics. However, the end goal in robotics is often not just to perform some computation, but to control real physical\nhardware such as a robot arm."),(0,n.kt)("p",null,"Interfacing with hardware peripherals imposes additional challenges, including:"),(0,n.kt)("ul",null,(0,n.kt)("li",{parentName:"ul"},"networking and communication beyond the ROS RMW publisher / subscriber mechanism"),(0,n.kt)("li",{parentName:"ul"},"hardware-specific control interfaces and requirements"),(0,n.kt)("li",{parentName:"ul"},"safety and reliability around controller and hardware limits"),(0,n.kt)("li",{parentName:"ul"},"potential real-time requirements for deterministic control performance")),(0,n.kt)("p",null,"The ",(0,n.kt)("inlineCode",{parentName:"p"},"ros2_control")," project defines an open standard for writing controllers and hardware interfaces with the aim of\nsimplifying the integration of new and existing robot hardware within a ROS-based applications through hardware\nabstraction. It specifies the following key concepts."),(0,n.kt)("h2",{id:"controller-manager"},"Controller Manager"),(0,n.kt)("p",null,"The idea behind hardware abstraction is to hide the additional complexity of hardware-specific software behind a\nstandardized API. The Controller Manager is a process that handles hardware interfaces and controllers and passes\nmessages between them in a standard format. In brief, the Controller Manager allows controllers in the ROS 2 graph to\nread the ",(0,n.kt)("strong",{parentName:"p"},"state")," of a robot and to write a ",(0,n.kt)("strong",{parentName:"p"},"command")," to the robot in a real-time loop."),(0,n.kt)("h3",{id:"state-interfaces"},"State interfaces"),(0,n.kt)("p",null,"State interfaces provide the current state of the robot. This is most commonly defined as the robot joint positions,\nand may also include joint velocities, torques or other available sensor data."),(0,n.kt)("h3",{id:"command-interfaces"},"Command interfaces"),(0,n.kt)("p",null,"Command interfaces represent the desired control action that a robot should execute. The nature of a command interface\ndepends on the robot actuators, but commonly includes joint positions, velocities or torques. For real-time control,\nvelocity and torque are the preferred command modes. For other actuators such as robot grippers, the command interface\nmight instead specify a gripper distance or force."),(0,n.kt)("h2",{id:"hardware-interfaces"},"Hardware interfaces"),(0,n.kt)("p",null,"Communication protocols and message standards vary between robot brands. Hardware interfaces, also known as hardware\ncomponents or hardware plugins, are software packages that implement robot-specific communication drivers. They are\nresponsible for reading data from the robot and translating them into the standard state interface format for the\nController Manager. They are also responsible for translating command interfaces from the Controller Manager into real\nrobot control actions."),(0,n.kt)("h2",{id:"controllers"},"Controllers"),(0,n.kt)("p",null,"As might be expected, controllers are responsible for calculating and sending desired control actions to the Controller\nManager via the standardized command interfaces. They also have access to state interfaces from the Controller Manager\nto use as feedback in closed-loop control."),(0,n.kt)("p",null,"Controllers are otherwise similar to ROS 2 lifecycle components; they have parameters, can publish or subscribe to\nmessages on the ROS 2 network to interact with other processes in the application, and can be dynamically loaded,\nactivated, deactivated or unloaded."),(0,n.kt)("admonition",{type:"info"},(0,n.kt)("p",{parentName:"admonition"},"Read more about ",(0,n.kt)("inlineCode",{parentName:"p"},"ros2_control"),"\nin ",(0,n.kt)("a",{parentName:"p",href:"https://control.ros.org/master/index.html"},"the official ros2_control documentation"),".")))}m.isMDXComponent=!0}}]);