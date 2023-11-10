"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[691],{3905:(e,n,t)=>{t.d(n,{Zo:()=>s,kt:()=>h});var a=t(7294);function o(e,n,t){return n in e?Object.defineProperty(e,n,{value:t,enumerable:!0,configurable:!0,writable:!0}):e[n]=t,e}function r(e,n){var t=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);n&&(a=a.filter((function(n){return Object.getOwnPropertyDescriptor(e,n).enumerable}))),t.push.apply(t,a)}return t}function l(e){for(var n=1;n<arguments.length;n++){var t=null!=arguments[n]?arguments[n]:{};n%2?r(Object(t),!0).forEach((function(n){o(e,n,t[n])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(t)):r(Object(t)).forEach((function(n){Object.defineProperty(e,n,Object.getOwnPropertyDescriptor(t,n))}))}return e}function i(e,n){if(null==e)return{};var t,a,o=function(e,n){if(null==e)return{};var t,a,o={},r=Object.keys(e);for(a=0;a<r.length;a++)t=r[a],n.indexOf(t)>=0||(o[t]=e[t]);return o}(e,n);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);for(a=0;a<r.length;a++)t=r[a],n.indexOf(t)>=0||Object.prototype.propertyIsEnumerable.call(e,t)&&(o[t]=e[t])}return o}var p=a.createContext({}),d=function(e){var n=a.useContext(p),t=n;return e&&(t="function"==typeof e?e(n):l(l({},n),e)),t},s=function(e){var n=d(e.components);return a.createElement(p.Provider,{value:n},e.children)},c="mdxType",m={inlineCode:"code",wrapper:function(e){var n=e.children;return a.createElement(a.Fragment,{},n)}},u=a.forwardRef((function(e,n){var t=e.components,o=e.mdxType,r=e.originalType,p=e.parentName,s=i(e,["components","mdxType","originalType","parentName"]),c=d(t),u=o,h=c["".concat(p,".").concat(u)]||c[u]||m[u]||r;return t?a.createElement(h,l(l({ref:n},s),{},{components:t})):a.createElement(h,l({ref:n},s))}));function h(e,n){var t=arguments,o=n&&n.mdxType;if("string"==typeof e||o){var r=t.length,l=new Array(r);l[0]=u;var i={};for(var p in n)hasOwnProperty.call(n,p)&&(i[p]=n[p]);i.originalType=e,i[c]="string"==typeof e?e:o,l[1]=i;for(var d=2;d<r;d++)l[d]=t[d];return a.createElement.apply(null,l)}return a.createElement.apply(null,t)}u.displayName="MDXCreateElement"},3007:(e,n,t)=>{t.r(n),t.d(n,{assets:()=>p,contentTitle:()=>l,default:()=>m,frontMatter:()=>r,metadata:()=>i,toc:()=>d});var a=t(7462),o=(t(7294),t(3905));const r={sidebar_position:1},l="YAML application syntax",i={unversionedId:"reference/yaml-syntax",id:"reference/yaml-syntax",title:"YAML application syntax",description:"The following sections define the YAML syntax used to describe an AICA application.",source:"@site/docs/reference/02-yaml-syntax.md",sourceDirName:"reference",slug:"/reference/yaml-syntax",permalink:"/docs/reference/yaml-syntax",draft:!1,editUrl:"https://github.com/aica-technology/api/tree/main/docs/docs/reference/02-yaml-syntax.md",tags:[],version:"current",sidebarPosition:1,frontMatter:{sidebar_position:1},sidebar:"programmingReferenceSidebar",previous:{title:"Programming introduction",permalink:"/docs/reference/intro"},next:{title:"Custom components",permalink:"/docs/category/custom-components"}},p={},d=[{value:"Overview",id:"overview",level:2},{value:"On start",id:"on-start",level:2},{value:"Hardware",id:"hardware",level:2},{value:"URDF",id:"urdf",level:3},{value:"Rate",id:"rate",level:3},{value:"Parameters",id:"parameters",level:3},{value:"Display name",id:"display-name",level:3},{value:"Position",id:"position",level:3},{value:"Controllers",id:"controllers",level:3},{value:"Conditions",id:"conditions",level:2},{value:"Simple conditions",id:"simple-conditions",level:3},{value:"Conditional operators",id:"conditional-operators",level:3},{value:"Not",id:"not",level:4},{value:"All",id:"all",level:4},{value:"Any",id:"any",level:4},{value:"One of",id:"one-of",level:4},{value:"Nested conditions",id:"nested-conditions",level:3},{value:"Components",id:"components",level:2},{value:"Component",id:"component",level:3},{value:"Display name",id:"display-name-1",level:3},{value:"Position",id:"position-1",level:3},{value:"Log level",id:"log-level",level:3},{value:"Mapping",id:"mapping",level:3},{value:"Parameters",id:"parameters-1",level:3},{value:"Component rate",id:"component-rate",level:4},{value:"Inputs and outputs",id:"inputs-and-outputs",level:3},{value:"Events",id:"events",level:3},{value:"Load or unload a component",id:"load-or-unload-a-component",level:4},{value:"Transition from one component to another",id:"transition-from-one-component-to-another",level:4},{value:"Trigger a lifecycle transition",id:"trigger-a-lifecycle-transition",level:4},{value:"Set a parameter",id:"set-a-parameter",level:4},{value:"Call a service",id:"call-a-service",level:4},{value:"Load or unload a hardware interface",id:"load-or-unload-a-hardware-interface",level:4},{value:"Load or unload a controller",id:"load-or-unload-a-controller",level:4},{value:"Activate or deactivate a controller",id:"activate-or-deactivate-a-controller",level:4},{value:"Special event predicates",id:"special-event-predicates",level:3},{value:"on_load",id:"on_load",level:4},{value:"on_unload",id:"on_unload",level:4},{value:"Validating a YAML application",id:"validating-a-yaml-application",level:2}],s={toc:d},c="wrapper";function m(e){let{components:n,...r}=e;return(0,o.kt)(c,(0,a.Z)({},s,r,{components:n,mdxType:"MDXLayout"}),(0,o.kt)("h1",{id:"yaml-application-syntax"},"YAML application syntax"),(0,o.kt)("p",null,"The following sections define the YAML syntax used to describe an AICA application."),(0,o.kt)("h2",{id:"overview"},"Overview"),(0,o.kt)("p",null,"An application description contains the following top-level elements."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"on_start:\n  ...\n\nhardware:\n  ...\n\nconditions:\n  ...\n\ncomponents:\n  ...\n")),(0,o.kt)("h2",{id:"on-start"},"On start"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"on_start")," keyword is reserved as a special event trigger when the application is launched.\nList the ",(0,o.kt)("a",{parentName:"p",href:"#events"},"events")," to trigger on startup (for example, to load components and hardware interfaces)."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"on_start:\n  load:\n    - component: component_a\n    - component: component_b\n    - hardware: robot_a\n")),(0,o.kt)("h2",{id:"hardware"},"Hardware"),(0,o.kt)("p",null,"Hardware interfaces describe the connected robots and their corresponding controllers."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"hardware:\n  robot_a:\n    urdf: ...\n    rate: ...\n    parameters:       # optional\n      ...\n    display_name: ... # optional\n    position: ...     # optional\n    controllers:\n      ...\n  robot_b:\n    ...\n")),(0,o.kt)("h3",{id:"urdf"},"URDF"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"urdf")," field refers to a specially formatted robot description file which defines the joint configurations and the\nhardware interface driver needed to communicate with the robot."),(0,o.kt)("p",null,"A hardware interface can be linked to URDF file in one of the following ways:"),(0,o.kt)("ul",null,(0,o.kt)("li",{parentName:"ul"},"By name of the custom URDF uploaded to the AICA database"),(0,o.kt)("li",{parentName:"ul"},"By name of an example URDF included in the AICA image (available examples depend on license and distribution versions)"),(0,o.kt)("li",{parentName:"ul"},"By the path of a URDF file mounted in the container filesystem"),(0,o.kt)("li",{parentName:"ul"},"By URDF string content inserted directly in the YAML (not recommended for large files)")),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},'# referring to a custom robot description uploaded to the user database\nrobot_a:\n  urdf: My custom robot\n\n# referring to a built-in robot description from the included examples\nrobot_b:\n  urdf: Universal Robots 5e (default configuration)\n\n# using the path to a URDF file mounted in the container filesystem\nrobot_c:\n  urdf: /home/ros2/my_robot.urdf\n\n# defining the URDF content in-line\nrobot_d:\n  urdf: |\n    <robot name="example">\n        <ros2_control name="ExampleRobotHardwareInterface" type="system">\n            <hardware>\n                <plugin>robot_interface/GenericInterface</plugin>\n            </hardware>\n            ...\n        </ros2_control>\n        ...\n    </robot>\n')),(0,o.kt)("admonition",{type:"info"},(0,o.kt)("p",{parentName:"admonition"},"Use the Hardware tab in the Developer Interface to manage available URDFs."),(0,o.kt)("p",{parentName:"admonition"},"Alternatively, use the API endpoints at ",(0,o.kt)("inlineCode",{parentName:"p"},"/v1/data/hardware")," and ",(0,o.kt)("inlineCode",{parentName:"p"},"/v1/examples/hardware")," to manage custom hardware and\nview the available built-in example URDFs, respectively.")),(0,o.kt)("h3",{id:"rate"},"Rate"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"rate")," field defines the robot control frequency in Hz."),(0,o.kt)("h3",{id:"parameters"},"Parameters"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"parameters")," field is used to set hardware-specific parameter values which override the default values from the\nassociated URDF. "),(0,o.kt)("p",null,"Specifically, the URDF is expected to include a ",(0,o.kt)("inlineCode",{parentName:"p"},"<ros2_control>")," tag under which hardware properties are defined,\nincluding the hardware plugin and any number of parameters specific to that plugin."),(0,o.kt)("p",null,"For example, a ",(0,o.kt)("inlineCode",{parentName:"p"},"robot_interface/GenericInterface")," plugin may accept a ",(0,o.kt)("inlineCode",{parentName:"p"},"robot_ip")," parameter to specify the IP address:"),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-xml"},'<robot name="example">\n    <ros2_control name="ExampleRobotHardwareInterface" type="system">\n        <hardware>\n            <plugin>robot_interface/GenericInterface</plugin>\n            <param name="robot_ip">192.168.0.1</param>\n        </hardware>\n        ...\n    </ros2_control>\n    ...\n</robot>\n')),(0,o.kt)("p",null,"By adding ",(0,o.kt)("inlineCode",{parentName:"p"},"robot_ip")," under the ",(0,o.kt)("inlineCode",{parentName:"p"},"parameters")," field, the default IP address can be overridden when the hardware interface\nis loaded:"),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"my_robot:\n  urdf: Example Robot\n  parameters:\n    robot_ip: 172.16.0.1\n")),(0,o.kt)("p",null,"In this example, the robot interface would be loaded with the IP address of ",(0,o.kt)("inlineCode",{parentName:"p"},"172.16.0.1")," instead of the default\n",(0,o.kt)("inlineCode",{parentName:"p"},"192.168.0.1")," as specified in the URDF. This allows parameters to be selectively altered at deploy time directly in the\napplication description without needing to modify the URDF itself."),(0,o.kt)("admonition",{type:"note"},(0,o.kt)("p",{parentName:"admonition"},"Hardware parameter values are only applied if the parameter name matches an existing hardware parameter in the URDF.\nIf the parameter does not exist in the URDF, it will not be added.")),(0,o.kt)("h3",{id:"display-name"},"Display name"),(0,o.kt)("p",null,"The optional ",(0,o.kt)("inlineCode",{parentName:"p"},"display_name")," field can be used to give the hardware interface a more human-readable name (one that\ndoesn't have to conform to the ",(0,o.kt)("inlineCode",{parentName:"p"},"lower_snake_case")," naming convention of the YAML syntax). It is only used when rendering\nthe hardware interface as a node in the AICA interactive graph editor. If omitted, the name is taken directly from the\nYAML field (from the previous example, it would default to ",(0,o.kt)("inlineCode",{parentName:"p"},"robot_a"),")."),(0,o.kt)("h3",{id:"position"},"Position"),(0,o.kt)("p",null,"The optional ",(0,o.kt)("inlineCode",{parentName:"p"},"position")," field is used to define the desired location of the hardware interface when rendered as a node\nin the AICA interactive graph editor. It has two subfields defining the X and Y location, respectively."),(0,o.kt)("p",null,"This field only affects visualization of the application graph and has no other run-time effect.\nIf a position is not specified, the node will be rendered at a procedurally chosen location."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"my_robot:\n  position:\n    x: 100\n    y: 200\n")),(0,o.kt)("h3",{id:"controllers"},"Controllers"),(0,o.kt)("p",null,"Controllers are the interface between components in the application and hardware in the real world. They convert desired\nreference signals into real joint commands according to some internal control law, and convert joint states from the\nrobot back to signals."),(0,o.kt)("p",null,"Controllers are listed under a top-level ",(0,o.kt)("inlineCode",{parentName:"p"},"controllers")," field. Controller names must be unique within the given hardware\ninterface, and should generally follow the ",(0,o.kt)("inlineCode",{parentName:"p"},"lower_camel_case")," naming convention."),(0,o.kt)("p",null,"Under each controller, the ",(0,o.kt)("inlineCode",{parentName:"p"},"plugin")," field refers to a registered controller plugin name."),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"parameters")," field then refers to configurable parameters for the given controller."),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"inputs")," and ",(0,o.kt)("inlineCode",{parentName:"p"},"outputs")," fields define the ROS2 topics to which each signal of the controller should be connected.\nSee also ",(0,o.kt)("a",{parentName:"p",href:"#inputs-and-outputs"},"Component Inputs and Outputs"),"."),(0,o.kt)("p",null,"Optionally, the ",(0,o.kt)("inlineCode",{parentName:"p"},"position")," field can be used to specify an X, Y location for rendering the hardware interface\nas a node in the AICA interactive graph editor. See also ",(0,o.kt)("a",{parentName:"p",href:"#position"},"Component Position"),"."),(0,o.kt)("p",null,"For example:"),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},'robot:\n  controllers:\n    broadcaster:\n      plugin: joint_state_broadcaster/JointStateBroadcaster\n    twist_controller:\n      plugin: "compliant_twist_controller/CompliantTwistController"\n      parameters:\n        linear_principle_damping: 10.0\n        linear_orthogonal_damping: 10.0\n        angular_stiffness: 1.0\n        angular_damping: { a: 1.0, b: true }\n      inputs:\n        command: /motion_generator/command_output\n      outputs:\n        state: /recorder/state_input\n')),(0,o.kt)("h2",{id:"conditions"},"Conditions"),(0,o.kt)("p",null,"Conditions are event triggers based on logical combinations of predicates."),(0,o.kt)("p",null,"Conditions are listed under a top-level field called ",(0,o.kt)("inlineCode",{parentName:"p"},"conditions"),". Condition names must be unique, and should\ngenerally follow the ",(0,o.kt)("inlineCode",{parentName:"p"},"lower_camel_case")," naming convention."),(0,o.kt)("p",null,"Conditional events are triggered only on the rising edge of the condition, preventing the repeated execution of an\nevent if the condition stays true."),(0,o.kt)("p",null,"Define events to be triggered by a condition by listing them under the condition name. See the ",(0,o.kt)("a",{parentName:"p",href:"#events"},"events")," section\nfor available event syntax."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"conditions:\n  condition_1:\n    component: ...\n    predicate: ...\n    events:\n      ...\n\n  condition_2:\n    <conditional_operator>: ...  # not, all, any, one_of\n    events:\n      ...\n\n")),(0,o.kt)("h3",{id:"simple-conditions"},"Simple conditions"),(0,o.kt)("p",null,"A simple condition evaluates just a single component predicate and triggers the listed events when it is true."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"condition_1:\n  component: my_component\n  predicate: some_component_predicate\n  events:\n    ...\n")),(0,o.kt)("h3",{id:"conditional-operators"},"Conditional operators"),(0,o.kt)("p",null,"To combine multiple predicates together into a single true / false condition, the following operators can be used."),(0,o.kt)("p",null,"The operators can refer to one or more component predicates with the syntax\n",(0,o.kt)("inlineCode",{parentName:"p"},"{ component: component_a, predicate: some_predicate }")),(0,o.kt)("h4",{id:"not"},"Not"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"not")," operator takes a single item and negates its value. It is true when the item is false, and false when the\nitem is true."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"condition_1:\n  not: { component: component_a, predicate: some_predicate }\n  events:\n    ...\n")),(0,o.kt)("h4",{id:"all"},"All"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"all")," operator takes a list of items and is true only when every listed item is true."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"condition_1:\n  all:\n    - { component: component_a, predicate: some_predicate }\n    - { component: component_b, predicate: some_predicate }\n  events:\n    ...\n")),(0,o.kt)("h4",{id:"any"},"Any"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"any")," operator takes a list of items and is true when at least one of the listed items is true."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"condition_1:\n  any:\n    - { component: component_a, predicate: some_predicate }\n    - { component: component_b, predicate: some_predicate }\n  events:\n    ...\n")),(0,o.kt)("h4",{id:"one-of"},"One of"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"one_of")," operator takes a list of items and is true only when exactly one of the listed items is true."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"condition_1:\n  one_of:\n    - { component: component_a, predicate: some_predicate }\n    - { component: component_b, predicate: some_predicate }\n  events:\n    ...\n")),(0,o.kt)("h3",{id:"nested-conditions"},"Nested conditions"),(0,o.kt)("p",null,"The conditional operators can be applied recursively for more complex conditions. The following example could be\ncollapsed into the equivalent logical pseudocode: ",(0,o.kt)("inlineCode",{parentName:"p"},"NOT(a AND b AND (c OR d OR (e XOR f)))")),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"conditions:\n  nested_condition:\n    not:\n      all:\n        - { component: component_1, predicate: a }\n        - { component: component_2, predicate: b }\n        - any:\n            - { component: component_3, predicate: c }\n            - { component: component_4, predicate: d }\n            - one_of:\n                - { component: component_5, predicate: e }\n                - { component: component_6, predicate: f }\n")),(0,o.kt)("h2",{id:"components"},"Components"),(0,o.kt)("p",null,"Components are listed under a top-level field called ",(0,o.kt)("inlineCode",{parentName:"p"},"components"),". Component names must be unique, and should\ngenerally follow the ",(0,o.kt)("inlineCode",{parentName:"p"},"lower_camel_case")," naming convention."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"components:\n  component_a:\n    component: ...    # required\n    display_name: ... # optional\n    position: ...     # optional\n    log_level: ...    # optional\n    mapping: ...      # optional\n    parameters: ...   # optional\n    inputs: ...       # optional\n    outputs: ...      # optional\n    events: ...       # optional\n\n  component_b:\n    ...\n")),(0,o.kt)("p",null,"Each component is defined with a number of fields, as shown below. The fields are defined in the next section."),(0,o.kt)("h3",{id:"component"},"Component"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"component")," field defines the actual component implementation to use for the component.\nIt takes a fully qualified class name as registered by the ",(0,o.kt)("inlineCode",{parentName:"p"},"RCLCPP_COMPONENTS_REGISTER_NODE")," macro."),(0,o.kt)("p",null,"The registered class name of a component should include the package name within the namespace. For example, the\nregistration ",(0,o.kt)("inlineCode",{parentName:"p"},"foo_components::Foo")," refers to a component ",(0,o.kt)("inlineCode",{parentName:"p"},"Foo")," in package ",(0,o.kt)("inlineCode",{parentName:"p"},"foo_components"),"."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"my_component:\n  component: foo_components::Foo\n")),(0,o.kt)("h3",{id:"display-name-1"},"Display name"),(0,o.kt)("p",null,"This optional field is identical to the ",(0,o.kt)("a",{parentName:"p",href:"#display-name"},"hardware display name")," and is used to assign a\nnicer, human-readable display name to the component when rendered as a node in the AICA interactive graph editor."),(0,o.kt)("h3",{id:"position-1"},"Position"),(0,o.kt)("p",null,"This optional field is identical to the ",(0,o.kt)("a",{parentName:"p",href:"#position"},"hardware position")," and is used to provide an X, Y position for the\ncomponent when rendered as a node in the AICA interactive graph editor."),(0,o.kt)("h3",{id:"log-level"},"Log level"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"log_level")," optionally sets the log severity level for this component.\nSupported levels are: ","[unset, debug, info, warn, error, fatal]"),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"my_component:\n  log_level: debug\n")),(0,o.kt)("h3",{id:"mapping"},"Mapping"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"mapping")," field optionally defines overrides for the component name and namespace. Normally, the component node\nis instantiated with the same name as the top level component name and put on the base namespace."),(0,o.kt)("p",null,"By specifying a mapping ",(0,o.kt)("inlineCode",{parentName:"p"},"name")," or ",(0,o.kt)("inlineCode",{parentName:"p"},"namespace")," or both, the instantiated node name is updated accordingly."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"# Without the mapping directive, the node name becomes /component_a\ncomponent_a:\n  ...\n\n# With the mapping directive, the node name becomes /my_component_namespace/my_new_component_name\ncomponent_b:\n  mapping:\n    name: my_new_component_name\n    namespace: my_component_namespace\n")),(0,o.kt)("h3",{id:"parameters-1"},"Parameters"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"parameters")," field allows initial parameters values to be set using a ",(0,o.kt)("inlineCode",{parentName:"p"},"name: value")," syntax.\nCurrently, only string and double parameters are supported. These values are only applied when the component\nis loaded and are not dynamically reconfigurable."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},'my_component:\n  parameters:\n    my_string_parameter: "my_string_value"\n    my_double_parameter: 2.0\n')),(0,o.kt)("h4",{id:"component-rate"},"Component rate"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"rate")," parameter is a special reserved parameter that defines the step rate of a component in Hertz, which is\nthe inverse of the execution period."),(0,o.kt)("p",null,"For example, if an image processing component should run some computation at 20 frames per second, then the\nrate parameter should be set to 20 Hertz."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"my_component:\n  parameters:\n    rate: 20\n")),(0,o.kt)("h3",{id:"inputs-and-outputs"},"Inputs and outputs"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"inputs")," and ",(0,o.kt)("inlineCode",{parentName:"p"},"outputs")," fields are used to connect component signals together to enable communication, signal\nprocessing and control loops. Each signal is specified using a ",(0,o.kt)("inlineCode",{parentName:"p"},"name: value")," syntax, where the name is the name\nof the signal according to the component description, and the value is and the name of the signal topic.\nIf a component output is assigned to the same topic name as another component input, they are connected, as\nillustrated in the example below."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},'my_component:\n  inputs:\n    robot_state: "/state"\n    applied_force: "/force"\n  outputs:\n    robot_command: "/command"\n\nmy_other_component:\n  outputs:\n    force_torque_sensor: "/force"\n')),(0,o.kt)("h3",{id:"events"},"Events"),(0,o.kt)("p",null,"Events drive the emergent behaviour of an application. Define events to be triggered by a predicate by listing them\nunder the predicate name, as shown below."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"my_component:\n  events:\n    is_active:\n      <triggered event_a>: ...\n      <triggered event_b>: ...\n    some_other_predicate_name:\n      <triggered event_c>: ...\n      <triggered event_bd>: ...\n")),(0,o.kt)("p",null,"Read more about ",(0,o.kt)("a",{parentName:"p",href:"/docs/concepts/building-blocks/events"},"events in the Concepts guide"),"."),(0,o.kt)("p",null,"The following events are defined."),(0,o.kt)("h4",{id:"load-or-unload-a-component"},"Load or unload a component"),(0,o.kt)("p",null,"Components can be loaded or unloaded by component name."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"load:\n  component: <component_name>\nunload:\n  component: <component_name>\n")),(0,o.kt)("p",null,"It is possible to load or unload multiple components simultaneously by specifying a list of components."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"load:\n  - component: component_a\n  - component: component_b\n")),(0,o.kt)("h4",{id:"transition-from-one-component-to-another"},"Transition from one component to another"),(0,o.kt)("p",null,'Component A can invoke a transition to component B as a shorthand for "unload component A, load component B".'),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"transition: <component_name>\n")),(0,o.kt)("h4",{id:"trigger-a-lifecycle-transition"},"Trigger a lifecycle transition"),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},'lifecycle: "configure"\n')),(0,o.kt)("p",null,"Request a lifecycle transition on the component that is triggering the event, using one of the available transitions\n(",(0,o.kt)("inlineCode",{parentName:"p"},"configure"),", ",(0,o.kt)("inlineCode",{parentName:"p"},"activate"),", ",(0,o.kt)("inlineCode",{parentName:"p"},"deactivate"),", ",(0,o.kt)("inlineCode",{parentName:"p"},"cleanup"),", or ",(0,o.kt)("inlineCode",{parentName:"p"},"shutdown"),")."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"lifecycle: activate\n")),(0,o.kt)("p",null,"Request a lifecycle transition on a different component."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"lifecycle:\n  transition: activate\n  component: <component_name>\n")),(0,o.kt)("p",null,"Use a list to trigger multiple transitions from a single predicate."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"lifecycle:\n  - transition: activate\n    component: <component_name>\n  - transition: deactivate\n    component: <component_name>\n")),(0,o.kt)("h4",{id:"set-a-parameter"},"Set a parameter"),(0,o.kt)("p",null,"Set a parameter on the component that is triggering the event."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"set:\n  parameter: <parameter_name>\n  value: <parameter_value>\n")),(0,o.kt)("p",null,"Set a parameter on a different component."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"set:\n  parameter: <parameter_name>\n  value: <parameter_value>\n  component: <component_name>\n")),(0,o.kt)("p",null,"Set a parameter on the controller of a particular hardware interface."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"set:\n  parameter: <parameter_name>\n  value: <parameter_value>\n  controller: <controller_name>\n  hardware: <hardware_name>\n")),(0,o.kt)("h4",{id:"call-a-service"},"Call a service"),(0,o.kt)("p",null,"Call a service with no payload on the component that is triggering the event."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"call_service: <service_name>\n")),(0,o.kt)("p",null,"Call a service on a different component."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"call_service:\n  service: <service_name>\n  component: <component_name>\n")),(0,o.kt)("p",null,"Call a service with a string payload."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},'call_service:\n  service: <service_name>\n  component: <component_name>\n  payload: "..."\n')),(0,o.kt)("p",null,"The service payload can also be written as any standard YAML object. The application parser will automatically encode\nthe object into a string format when making the service call. In this case, the component service is responsible\nfor parsing the string back into a YAML object, dict or structure as necessary."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},'call_service:\n  service: <service_name>\n  component: <component_name>\n  payload:\n    foo: "some content"\n    bar: [ x, y, z ]\n    baz:\n      a: 1\n      b: 2\n')),(0,o.kt)("h4",{id:"load-or-unload-a-hardware-interface"},"Load or unload a hardware interface"),(0,o.kt)("p",null,"Load and initialize a hardware interface."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"load:\n  hardware: <hardware_name>\n")),(0,o.kt)("p",null,"Unload and destroy a hardware interface."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"unload:\n  hardware: <hardware_name>\n")),(0,o.kt)("admonition",{type:"caution"},(0,o.kt)("p",{parentName:"admonition"},"All hardware interfaces in the application are automatically loaded and initialized when the application starts."),(0,o.kt)("p",{parentName:"admonition"},"This behavior may change in the near future.")),(0,o.kt)("h4",{id:"load-or-unload-a-controller"},"Load or unload a controller"),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"load:\n  hardware: <hardware_name>\n  controller: <controller_name>\n\nunload:\n  hardware: <hardware_name>\n  controller: <controller_name>\n")),(0,o.kt)("p",null,"Use a list to load or unload multiple controllers from a single predicate."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"load:\n  - hardware: <hardware_name>\n    controller: controller_a\n  - hardware: <hardware_name>\n    controller: controller_b\n")),(0,o.kt)("h4",{id:"activate-or-deactivate-a-controller"},"Activate or deactivate a controller"),(0,o.kt)("p",null,"Use the ",(0,o.kt)("inlineCode",{parentName:"p"},"switch_controllers")," event to list the controllers to be activated or deactivated for a specific hardware\ninterface."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},'switch_controllers:\n  hardware: <hardware_name>\n  activate: [ <controller_name>, <controller_name> ]\n  deactivate: [ "controller_three", "controller_four" ] \n')),(0,o.kt)("admonition",{type:"note"},(0,o.kt)("p",{parentName:"admonition"},"A controller must be loaded before it can be activated, and must be deactivated before it can be unloaded.")),(0,o.kt)("h3",{id:"special-event-predicates"},"Special event predicates"),(0,o.kt)("h4",{id:"on_load"},"on_load"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"on_load")," predicate is provided by the state engine and set to true after the component\nhas been loaded. Any events associated with the ",(0,o.kt)("inlineCode",{parentName:"p"},"on_load")," predicate are handled once\non instantiation of the node."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"component:\n  events:\n    on_load:\n      <some triggered event>: ...\n")),(0,o.kt)("h4",{id:"on_unload"},"on_unload"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"on_unload")," predicate is similar to the ",(0,o.kt)("inlineCode",{parentName:"p"},"on_load")," predicate and is provided by the state engine.\nAny events associated with the ",(0,o.kt)("inlineCode",{parentName:"p"},"on_unload")," predicate are handled once upon destruction of the component interface."),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"component:\n  events:\n    on_unload:\n      <some triggered event>: ...\n")),(0,o.kt)("h2",{id:"validating-a-yaml-application"},"Validating a YAML application"),(0,o.kt)("p",null,"The ",(0,o.kt)("a",{target:"_blank",href:t(6513).Z},"YAML application schema")," defines the structural rules\nof an AICA application and effectively distinguishes between valid and invalid syntax."),(0,o.kt)("p",null,"Many modern IDEs and code editors can be configured to support custom schemas and provide in-line validation and\ncompletion of the YAML content."))}m.isMDXComponent=!0},6513:(e,n,t)=>{t.d(n,{Z:()=>a});const a=t.p+"assets/files/application.schema-bdefb05c00d643139f83f1e854eaa187.json"}}]);