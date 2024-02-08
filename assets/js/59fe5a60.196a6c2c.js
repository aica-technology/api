"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[547],{3905:(e,t,a)=>{a.d(t,{Zo:()=>s,kt:()=>u});var n=a(7294);function r(e,t,a){return t in e?Object.defineProperty(e,t,{value:a,enumerable:!0,configurable:!0,writable:!0}):e[t]=a,e}function o(e,t){var a=Object.keys(e);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);t&&(n=n.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),a.push.apply(a,n)}return a}function i(e){for(var t=1;t<arguments.length;t++){var a=null!=arguments[t]?arguments[t]:{};t%2?o(Object(a),!0).forEach((function(t){r(e,t,a[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(a)):o(Object(a)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(a,t))}))}return e}function l(e,t){if(null==e)return{};var a,n,r=function(e,t){if(null==e)return{};var a,n,r={},o=Object.keys(e);for(n=0;n<o.length;n++)a=o[n],t.indexOf(a)>=0||(r[a]=e[a]);return r}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(n=0;n<o.length;n++)a=o[n],t.indexOf(a)>=0||Object.prototype.propertyIsEnumerable.call(e,a)&&(r[a]=e[a])}return r}var c=n.createContext({}),p=function(e){var t=n.useContext(c),a=t;return e&&(a="function"==typeof e?e(t):i(i({},t),e)),a},s=function(e){var t=p(e.components);return n.createElement(c.Provider,{value:t},e.children)},d="mdxType",h={inlineCode:"code",wrapper:function(e){var t=e.children;return n.createElement(n.Fragment,{},t)}},m=n.forwardRef((function(e,t){var a=e.components,r=e.mdxType,o=e.originalType,c=e.parentName,s=l(e,["components","mdxType","originalType","parentName"]),d=p(a),m=r,u=d["".concat(c,".").concat(m)]||d[m]||h[m]||o;return a?n.createElement(u,i(i({ref:t},s),{},{components:a})):n.createElement(u,i({ref:t},s))}));function u(e,t){var a=arguments,r=t&&t.mdxType;if("string"==typeof e||r){var o=a.length,i=new Array(o);i[0]=m;var l={};for(var c in t)hasOwnProperty.call(t,c)&&(l[c]=t[c]);l.originalType=e,l[d]="string"==typeof e?e:r,i[1]=l;for(var p=2;p<o;p++)i[p]=a[p];return n.createElement.apply(null,i)}return n.createElement.apply(null,a)}m.displayName="MDXCreateElement"},924:(e,t,a)=>{a.r(t),a.d(t,{assets:()=>c,contentTitle:()=>i,default:()=>h,frontMatter:()=>o,metadata:()=>l,toc:()=>p});var n=a(7462),r=(a(7294),a(3905));const o={sidebar_position:3},i="An application with hardware",l={unversionedId:"getting-started/examples/mock-hardware-example",id:"getting-started/examples/mock-hardware-example",title:"An application with hardware",description:"Package requirements",source:"@site/docs/getting-started/05-examples/03-mock-hardware-example.md",sourceDirName:"getting-started/05-examples",slug:"/getting-started/examples/mock-hardware-example",permalink:"/docs/getting-started/examples/mock-hardware-example",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/getting-started/05-examples/03-mock-hardware-example.md",tags:[],version:"current",sidebarPosition:3,frontMatter:{sidebar_position:3},sidebar:"gettingStartedSidebar",previous:{title:"The application graph editor",permalink:"/docs/getting-started/examples/editor-example"},next:{title:"Next steps",permalink:"/docs/getting-started/next"}},c={},p=[{value:"Package requirements",id:"package-requirements",level:2},{value:"URDF Hardware Manager",id:"urdf-hardware-manager",level:2},{value:"Setting up the application",id:"setting-up-the-application",level:2},{value:"The example explained",id:"the-example-explained",level:2},{value:"Run the application",id:"run-the-application",level:2},{value:"Visualize the mock robot in RViz",id:"visualize-the-mock-robot-in-rviz",level:2}],s={toc:p},d="wrapper";function h(e){let{components:t,...o}=e;return(0,r.kt)(d,(0,n.Z)({},s,o,{components:t,mdxType:"MDXLayout"}),(0,r.kt)("h1",{id:"an-application-with-hardware"},"An application with hardware"),(0,r.kt)("h2",{id:"package-requirements"},"Package requirements"),(0,r.kt)("p",null,"This example requires the Universal Robots collection which includes example URDF content."),(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-toml",metastring:'title="aica-package.toml"',title:'"aica-package.toml"'},'#syntax=ghcr.io/aica-technology/app-builder:v1\n\n[packages]\n"@aica/base" = "v3.0.0"\n\n"@aica/collections/ur-collection" = "v3.0.4"\n')),(0,r.kt)("h2",{id:"urdf-hardware-manager"},"URDF Hardware Manager"),(0,r.kt)("p",null,"After starting the application container, open the Hardware tab of the Developer\nInterface (",(0,r.kt)("a",{parentName:"p",href:"http://localhost:8080/dev/hardware"},"localhost:8080/dev/hardware"),"). This page shows a table of available URDF\nfiles in the container database with a name and a description."),(0,r.kt)("p",null,'AICA hardware collections include example URDFs, which are shown on the table with a pad-lock icon indicating that they\nare not editable. Users can make an editable copy of a selected URDF with the "Save As" button, or upload and edit\ncustom URDFs.'),(0,r.kt)("admonition",{type:"note"},(0,r.kt)("p",{parentName:"admonition"},"Refer to the section on ",(0,r.kt)("a",{parentName:"p",href:"/docs/getting-started/run#persistent-user-data"},"Persistent user data")," on how to mount a container volume to\npersist database content between containers.")),(0,r.kt)("p",null,'From the hardware manager table, select the entry named "Universal Robots 5e (mock interface)". The URDF content should\nappear on the right side of the page.'),(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-xml",metastring:'title="Universal Robots 5e (mock interface)"',title:'"Universal',Robots:!0,"5e":!0,"(mock":!0,'interface)"':!0},'<?xml version="1.0" ?>\n<robot name="ur5e">\n    <ros2_control name="UniversalRobotsInterface" type="system">\n        <hardware>\n            #highlight-next-line\n            <plugin>robot_interface/MockInterface</plugin>\n        </hardware>\n        ...\n    </ros2_control>\n</robot>\n')),(0,r.kt)("p",null,"The selected URDF specifies the hardware plugin ",(0,r.kt)("inlineCode",{parentName:"p"},"robot_interface/MockInterface"),". This is a generic AICA plugin that\nmocks real robot hardware by perfectly following all commands and reflecting back the robot state."),(0,r.kt)("p",null,"The mock URDF will be used to demonstrate the hardware interface block in AICA applications."),(0,r.kt)("admonition",{type:"info"},(0,r.kt)("p",{parentName:"admonition"},"Refer to the overview section ",(0,r.kt)("a",{parentName:"p",href:"/docs/concepts/ros-concepts/control"},"Controlling robots with ros2_control")," for\nmore context.")),(0,r.kt)("h2",{id:"setting-up-the-application"},"Setting up the application"),(0,r.kt)("p",null,"Go to the Editor page using the top navigation bar or at ",(0,r.kt)("a",{parentName:"p",href:"http://localhost:8080/dev/editor"},"localhost:8080/dev/editor"),"\nand create a new application."),(0,r.kt)("p",null,"Enter the following YAML and generate the graph."),(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-yaml"},"on_start:\n  load:\n    - hardware: mock_hardware\n    - controller: robot_state_broadcaster\n      hardware: mock_hardware\nbuttons:\n  activate_controller:\n    position:\n      x: 0\n      y: 280\n    on_click:\n      switch_controllers:\n        hardware: mock_hardware\n        activate: robot_state_broadcaster\ncomponents: { }\nhardware:\n  mock_hardware:\n    display_name: Mock Hardware Interface\n    position:\n      x: 400\n      y: -80\n    urdf: Universal Robots 5e (mock interface)\n    rate: 60\n    controllers:\n      robot_state_broadcaster:\n        plugin: modulo_controllers/RobotStateBroadcaster\n")),(0,r.kt)("p",null,"The application graph should show a hardware interface with a controller and an event trigger."),(0,r.kt)("p",null,(0,r.kt)("img",{alt:"mock hardware example graph",src:a(3958).Z,width:"1114",height:"704"})),(0,r.kt)("h2",{id:"the-example-explained"},"The example explained"),(0,r.kt)("p",null,"Starting from the bottom, the top-level ",(0,r.kt)("inlineCode",{parentName:"p"},"hardware")," field defines the hardware interfaces in an application."),(0,r.kt)("p",null,"In this case, there is one hardware interface called ",(0,r.kt)("inlineCode",{parentName:"p"},"mock_hardware"),"."),(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-yaml"},"  mock_hardware:\n    display_name: Mock Hardware Interface\n    position:\n      x: 400\n      y: -80\n    urdf: Universal Robots 5e (mock interface)\n    controllers:\n      robot_state_broadcaster:\n        plugin: modulo_controllers/RobotStateBroadcaster\n")),(0,r.kt)("p",null,"The ",(0,r.kt)("inlineCode",{parentName:"p"},"urdf")," field specifies the ",(0,r.kt)("inlineCode",{parentName:"p"},"Universal Robots 5e (mock interface)")," URDF as identified on the hardware manager page."),(0,r.kt)("p",null,"The ",(0,r.kt)("inlineCode",{parentName:"p"},"controllers")," field lists the controllers associated with the hardware interface. In this example, the only\ncontroller is the ",(0,r.kt)("inlineCode",{parentName:"p"},"modulo_controllers/RobotStateBroadcaster"),", which is a generic AICA controller that broadcasts the\nrobot joint states and transforms."),(0,r.kt)("admonition",{type:"tip"},(0,r.kt)("p",{parentName:"admonition"},"Learn more about available properties for application hardware on\nthe ",(0,r.kt)("a",{parentName:"p",href:"/docs/reference/yaml-syntax"},"YAML application syntax")," reference page.")),(0,r.kt)("p",null,"Moving up the application, the ",(0,r.kt)("inlineCode",{parentName:"p"},"components")," field is left empty because there are no components in this example."),(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-yaml"},"components: { }\n")),(0,r.kt)("p",null,"Above that, the application defines an event trigger button under the top-level ",(0,r.kt)("inlineCode",{parentName:"p"},"buttons")," field."),(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-yaml"},"buttons:\n  activate_controller:\n    position:\n      x: 0\n      y: 280\n    on_click:\n      switch_controllers:\n        hardware: mock_hardware\n        activate: robot_state_broadcaster\n")),(0,r.kt)("p",null,"The ",(0,r.kt)("inlineCode",{parentName:"p"},"on_click")," field defines the application events that are triggered with the event button is pressed. In this case,\nit triggers the ",(0,r.kt)("inlineCode",{parentName:"p"},"switch_controllers")," event which is used to activate the ",(0,r.kt)("inlineCode",{parentName:"p"},"robot_state_broadcaster")," controller on the\n",(0,r.kt)("inlineCode",{parentName:"p"},"mock_hardware")," interface."),(0,r.kt)("p",null,"The application begins with the ",(0,r.kt)("inlineCode",{parentName:"p"},"on_start")," directive to list the initial application events."),(0,r.kt)("pre",null,(0,r.kt)("code",{parentName:"pre",className:"language-yaml"},"on_start:\n  load:\n    - hardware: mock_hardware\n    - controller: robot_state_broadcaster\n      hardware: mock_hardware\n")),(0,r.kt)("p",null,"In this case, the first event that occurs in the application is to load the ",(0,r.kt)("inlineCode",{parentName:"p"},"mock_hardware")," hardware interface.\nAfter that, the ",(0,r.kt)("inlineCode",{parentName:"p"},"robot_state_broadcaster")," controller is loaded."),(0,r.kt)("h2",{id:"run-the-application"},"Run the application"),(0,r.kt)("p",null,"Putting it all together, pressing Play on this application should load the mock hardware interface and load the\nbroadcaster controller. When the trigger button is pressed in the graph editor, the broadcaster will be activated."),(0,r.kt)("admonition",{type:"caution"},(0,r.kt)("p",{parentName:"admonition"},'The loading behavior of hardware interfaces and controllers may change in future versions to simplify "auto-load"\nprocedures similar\nto ',(0,r.kt)("a",{parentName:"p",href:"/docs/concepts/building-blocks/components#auto-lifecycle-events"},"component auto-lifecycle events"))),(0,r.kt)("h2",{id:"visualize-the-mock-robot-in-rviz"},"Visualize the mock robot in RViz"),(0,r.kt)("admonition",{type:"info"},(0,r.kt)("p",{parentName:"admonition"},"Only users with a Linux host can visualize the robot with RViz. Follow the steps in\nthe ",(0,r.kt)("a",{parentName:"p",href:"/docs/getting-started/run#display-sharing"},"Display sharing")," section to attach a new terminal to the running container.")),(0,r.kt)("p",null,"Open RViz in the container with the ",(0,r.kt)("inlineCode",{parentName:"p"},"rviz2")," command."),(0,r.kt)("p",null,"In the Displays panel under Global Options, set the Fixed Frame to ",(0,r.kt)("inlineCode",{parentName:"p"},"world"),"."),(0,r.kt)("p",null,"Press Add or CTRL+N to add a new display and select the RobotModel plugin."),(0,r.kt)("p",null,"Under the RobotModel Description Topic, enter ",(0,r.kt)("inlineCode",{parentName:"p"},"/mock_hardware/robot_description"),"."),(0,r.kt)("admonition",{type:"tip"},(0,r.kt)("p",{parentName:"admonition"},"The robot description topic will correspond to the name of the hardware interface in the YAML application.")),(0,r.kt)("p",null,"When the application is playing and the robot broadcaster controller has been activated, the robot model should appear\nin the RViz viewer."),(0,r.kt)("p",null,(0,r.kt)("img",{alt:"mock hardware example rviz",src:a(623).Z,width:"1862",height:"1187"})))}h.isMDXComponent=!0},3958:(e,t,a)=>{a.d(t,{Z:()=>n});const n=a.p+"assets/images/mock-hardware-example-graph-48a4afa88106b2acb58e55cf6f0cc18c.png"},623:(e,t,a)=>{a.d(t,{Z:()=>n});const n=a.p+"assets/images/mock-hardware-example-rviz-e0b1b45c9e544450cdf745db74241143.png"}}]);