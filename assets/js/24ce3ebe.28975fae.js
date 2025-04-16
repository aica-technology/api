"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[374],{2353:(e,n,a)=>{a.r(n),a.d(n,{assets:()=>d,contentTitle:()=>l,default:()=>p,frontMatter:()=>c,metadata:()=>i,toc:()=>h});const i=JSON.parse('{"id":"reference/manual-installation-launch","title":"Manual Installation and Launch","description":"The following sections explain how to install and launch AICA Core and any additional packages manually from the command","source":"@site/docs/reference/02-manual-installation-launch.md","sourceDirName":"reference","slug":"/reference/manual-installation-launch","permalink":"/docs/reference/manual-installation-launch","draft":false,"unlisted":false,"editUrl":"https://github.com/aica-technology/api/tree/main/docs/docs/reference/02-manual-installation-launch.md","tags":[],"version":"current","sidebarPosition":1,"frontMatter":{"sidebar_position":1,"title":"Manual Installation and Launch"},"sidebar":"programmingReferenceSidebar","previous":{"title":"Programming introduction","permalink":"/docs/reference/intro"},"next":{"title":"YAML application syntax","permalink":"/docs/reference/yaml-syntax"}}');var t=a(4848),o=a(8453),s=a(5537),r=a(9329);const c={sidebar_position:1,title:"Manual Installation and Launch"},l=void 0,d={},h=[{value:"Configuring your system when Docker Desktop for Linux is installed",id:"configuring-your-system-when-docker-desktop-for-linux-is-installed",level:2},{value:"Configuring Docker context",id:"configuring-docker-context",level:3},{value:"Logging in to the AICA package registry",id:"logging-in-to-the-aica-package-registry",level:2},{value:"Configuring AICA packages with a manifest file",id:"configuring-aica-packages-with-a-manifest-file",level:2},{value:"Configuring a minimal runtime image with a version of AICA Core",id:"configuring-a-minimal-runtime-image-with-a-version-of-aica-core",level:3},{value:"Configuring a runtime image with add-on packages",id:"configuring-a-runtime-image-with-add-on-packages",level:3},{value:"Including custom packages",id:"including-custom-packages",level:3},{value:"Building an AICA runtime application image",id:"building-an-aica-runtime-application-image",level:2},{value:"Starting the application container",id:"starting-the-application-container",level:2},{value:"Stopping the application container",id:"stopping-the-application-container",level:2},{value:"Persistent user data",id:"persistent-user-data",level:3},{value:"Setting a super-admin password",id:"setting-a-super-admin-password",level:3},{value:"Access the AICA Studio",id:"access-the-aica-studio",level:2},{value:"Access the REST API",id:"access-the-rest-api",level:2},{value:"Connect a terminal session to the container",id:"connect-a-terminal-session-to-the-container",level:2},{value:"Display sharing",id:"display-sharing",level:3}];function u(e){const n={a:"a",admonition:"admonition",code:"code",h2:"h2",h3:"h3",p:"p",pre:"pre",strong:"strong",...(0,o.R)(),...e.components};return(0,t.jsxs)(t.Fragment,{children:[(0,t.jsxs)(n.p,{children:["The following sections explain how to install and launch AICA Core and any additional packages manually from the command\nline without the use of AICA Launcher. The pre-requisites are still a valid license and a host with Docker installed.\nFor the rest of this guide, it will be assumed that a valid license has been saved to a file called ",(0,t.jsx)(n.code,{children:"aica-license.toml"}),"\non the host machine."]}),"\n",(0,t.jsx)(n.h2,{id:"configuring-your-system-when-docker-desktop-for-linux-is-installed",children:"Configuring your system when Docker Desktop for Linux is installed"}),"\n",(0,t.jsx)(n.p,{children:"If you are using Linux but do not have Docker Desktop for Linux installed, you may skip this section."}),"\n",(0,t.jsx)(n.p,{children:"For Linux users that have Docker Desktop for Linux installed, some additional steps may be required to ensure that\nAICA System software is used at its full potential."}),"\n",(0,t.jsxs)(n.p,{children:["The main issue originates from Docker Desktop for Linux creating a custom Docker context (",(0,t.jsx)(n.code,{children:"desktop-linux"}),") and endpoint\nto manage its images. This is rightfully done to encapsulate those images within uses of the Docker Desktop GUI, without\ncontaminating other parts of your system. However, creating a custom context with an endpoint in your ",(0,t.jsx)(n.code,{children:"/home"})," directory\nmeans you now can not use Docker with elevated privileges (e.g., external devices, forwarding graphics, ...) and that\nsome AICA Launcher functionalities will not work out-of-the-box (e.g., attaching to a terminal)."]}),"\n",(0,t.jsx)(n.h3,{id:"configuring-docker-context",children:"Configuring Docker context"}),"\n",(0,t.jsxs)(n.p,{children:[(0,t.jsx)(n.code,{children:"desktop-linux"})," will typically be the default context when starting up your system. To avoid the above limitations,\nmake sure to change the context before building or executing AICA applications, or running AICA Launcher."]}),"\n",(0,t.jsx)(n.p,{children:"See the available contexts on your system:"}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-shell",children:"docker context ls\n"})}),"\n",(0,t.jsxs)(n.p,{children:["You should see a ",(0,t.jsx)(n.code,{children:"default"})," choice alongside ",(0,t.jsx)(n.code,{children:"desktop-linux"})," that Docker Desktop for Linux created. If you do not see\nit, then you may have missed some of the installation steps (refer to the\n",(0,t.jsx)(n.a,{href:"/docs/getting-started/installation-and-launch",children:"installation guide"}),"). If it is indeed there, make sure it is active:"]}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-shell",children:"docker context use default\n"})}),"\n",(0,t.jsx)(n.admonition,{type:"note",children:(0,t.jsx)(n.p,{children:"You may need to repeat these steps upon a restart of your system."})}),"\n",(0,t.jsx)(n.h2,{id:"logging-in-to-the-aica-package-registry",children:"Logging in to the AICA package registry"}),"\n",(0,t.jsxs)(n.p,{children:["To authenticate docker to login and pull images from the registry, run the following command (no replacement of\n",(0,t.jsx)(n.code,{children:"USERNAME"})," required):"]}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-shell",children:"cat aica-license.toml | docker login registry.licensing.aica.tech -u USERNAME --password-stdin\n"})}),"\n",(0,t.jsx)(n.h2,{id:"configuring-aica-packages-with-a-manifest-file",children:"Configuring AICA packages with a manifest file"}),"\n",(0,t.jsxs)(n.p,{children:["A runtime application image is configured using a simple ",(0,t.jsx)(n.strong,{children:"manifest file"})," defining the version of AICA Core to use and\noptionally defining additional add-on packages. The manifest file contains a custom docker syntax header pointing to\nAICA's app-builder tool, and the ",(0,t.jsx)(n.code,{children:"docker build"})," command is used to bundle all listed packages into a final runtime\nimage."]}),"\n",(0,t.jsx)(n.h3,{id:"configuring-a-minimal-runtime-image-with-a-version-of-aica-core",children:"Configuring a minimal runtime image with a version of AICA Core"}),"\n",(0,t.jsxs)(n.p,{children:["The manifest file must contain a syntax header and a list of packages. The minimal version of the manifest includes\nonly AICA Core as the ",(0,t.jsx)(n.code,{children:"core"})," package. The version can be changed according to the available releases."]}),"\n",(0,t.jsx)(n.admonition,{type:"info",children:(0,t.jsxs)(n.p,{children:["In the past, you might have seen applications using the ",(0,t.jsx)(n.code,{children:"aica-package.toml"})," filename. While you can use any filename as\nwe do not enforce any, we recommend using ",(0,t.jsx)(n.code,{children:"aica-application.toml"})," to avoid confusion with the ",(0,t.jsx)(n.code,{children:"aica-package.toml"})," file\nwhich is used for building packages using ",(0,t.jsx)(n.code,{children:"package-builder"}),"."]})}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-toml",metastring:'title="aica-application.toml"',children:'#syntax=ghcr.io/aica-technology/app-builder:v2\n\n[core]\nimage = "v4.2.0"\n'})}),"\n",(0,t.jsx)(n.h3,{id:"configuring-a-runtime-image-with-add-on-packages",children:"Configuring a runtime image with add-on packages"}),"\n",(0,t.jsxs)(n.p,{children:["A manifest can include additional components and hardware collections as add-on packages. For any available package\nlisted in the AICA registry, specify the package and version with the ",(0,t.jsx)(n.code,{children:"@aica/"})," prefix. The following example manifest\nfile includes two add-on packages: version 2.0.0 of the ",(0,t.jsx)(n.code,{children:"components/rl_policy_components"})," component package and version\n4.1.0 of the ",(0,t.jsx)(n.code,{children:"collections/ur-collection"})," hardware collection package."]}),"\n",(0,t.jsx)(n.admonition,{type:"note",children:(0,t.jsxs)(n.p,{children:["Starting with version ",(0,t.jsx)(n.code,{children:"2.0.0"})," of the ",(0,t.jsx)(n.code,{children:"app-builder"}),", all packages need to have special metadata associated in their\nimage. This is done automatically when building with newer versions of ",(0,t.jsx)(n.code,{children:"app-builder"}),". This means you won't be able to\nuse older versions of certain libraries and packages with newer versions of ",(0,t.jsx)(n.code,{children:"app-builder"}),"."]})}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-toml",metastring:'title="aica-application.toml"',children:'#syntax=ghcr.io/aica-technology/app-builder:v2\n\n[core]\n"image" = "v4.2.0"\n\n[packages]\n# add components\n"@aica/components/rl-policy-components" = "v2.0.0"\n\n# add hardware collections\n"@aica/collections/ur-collection" = "v4.1.0"\n'})}),"\n",(0,t.jsx)(n.h3,{id:"including-custom-packages",children:"Including custom packages"}),"\n",(0,t.jsxs)(n.p,{children:["The AICA framework allows developers to build their own\n",(0,t.jsx)(n.a,{href:"/docs/reference/custom-components/component-package",children:"custom components"}),". These packages can be included under a\ncustom name using the ",(0,t.jsx)(n.code,{children:"docker-image://"})," prefix to specify the docker image name or path. For example, a custom component\npackage that was locally built using ",(0,t.jsx)(n.code,{children:"docker build [...] --tag my-custom-component-package"})," could be included as\n",(0,t.jsx)(n.code,{children:"docker-image://my-custom-component-package"}),". Community and third-party packages may also be available on other docker\nregistries such as DockerHub or GitHub Container Registry and can be included with the associated docker path."]}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-toml",metastring:'title="aica-application.toml"',children:'#syntax=ghcr.io/aica-technology/app-builder:v2\n\n[core]\n"image" = "v4.2.0"\n\n[packages]\n# add a custom package from a local docker image path\n"my-local-package" = "docker-image://my-custom-component-package"\n\n# add a package from any docker path such as GitHub Container Registry\n"my-ghcr-package" = "docker-image://ghcr.io/user/package:tag"\n'})}),"\n",(0,t.jsx)(n.h2,{id:"building-an-aica-runtime-application-image",children:"Building an AICA runtime application image"}),"\n",(0,t.jsx)(n.admonition,{type:"note",children:(0,t.jsxs)(n.p,{children:[(0,t.jsx)(n.a,{href:"#logging-in-to-the-aica-package-registry",children:"Log in to the package registry"})," before building the image to authorize docker\nto access AICA packages."]})}),"\n",(0,t.jsxs)(n.p,{children:["Once the desired packages have been configured in a manifest file, a ",(0,t.jsx)(n.code,{children:"docker build"})," command can be used to build the\nruntime application image. In this example, a manifest file saved as ",(0,t.jsx)(n.code,{children:"aica-application.toml"})," is used to build an image\nwith the name ",(0,t.jsx)(n.code,{children:"aica-runtime"}),"."]}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-shell",children:"docker build -f aica-application.toml -t aica-runtime .\n"})}),"\n",(0,t.jsxs)(n.p,{children:["The command ",(0,t.jsx)(n.code,{children:"docker image ls | grep aica-runtime"})," should then list the ",(0,t.jsx)(n.code,{children:"aica-runtime"})," image."]}),"\n",(0,t.jsx)(n.h2,{id:"starting-the-application-container",children:"Starting the application container"}),"\n",(0,t.jsx)(n.p,{children:"You can start the AICA application container with the following command."}),"\n",(0,t.jsx)(n.admonition,{type:"note",children:(0,t.jsxs)(n.p,{children:["Change ",(0,t.jsx)(n.code,{children:"/path/to/license"})," in the command below to the location of the ",(0,t.jsx)(n.code,{children:"aica-license.toml"})," file from above. For example,\nuse ",(0,t.jsx)(n.code,{children:"~/.aica-license.toml"})," to keep the license file hidden in the home folder."]})}),"\n",(0,t.jsxs)(s.A,{groupId:"os",children:[(0,t.jsx)(r.A,{value:"linux",label:"Linux",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"docker run -it --rm \\\n  --privileged \\\n  --net=host \\\n  -v /path/to/license:/license:ro \\\n  aica-runtime\n"})})}),(0,t.jsxs)(r.A,{value:"mac",label:"macOS",children:[(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"docker run -it --rm \\\n  --privileged \\\n  -p 8080:8080 -p 18000-18100:18000-18100/udp \\\n  -v /path/to/license:/license:ro \\\n  aica-runtime\n"})}),(0,t.jsx)(n.admonition,{type:"note",children:(0,t.jsxs)(n.p,{children:["If port 8080 is already used on the host, use ",(0,t.jsx)(n.code,{children:"-p HOST_PORT:8080"})," to avoid conflicts. Do not remap ports ",(0,t.jsx)(n.code,{children:"18000-18100"}),"."]})})]})]}),"\n",(0,t.jsx)(n.p,{children:"When the container starts up, it will generate some initial output in the terminal window that should look something\nlike the example below:"}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-console",children:"[2024-11-18 13:43:12 +0000] [87] [INFO] Starting gunicorn 21.2.0\n[2024-11-18 13:43:12 +0000] [87] [INFO] Listening at: http://0.0.0.0:5000 (87)\n[2024-11-18 13:43:12 +0000] [87] [INFO] Using worker: eventlet\n[2024-11-18 13:43:12 +0000] [100] [INFO] Booting worker with pid: 100\n[INFO] [rosapi_node-1]: process started with pid [151]\n[INFO] [1731937392.265880595] [EventEngine.ServiceHandler]: Initializing event engine services\n[INFO] [1731937392.270243234] [event_engine]: No initial application provided. Use the event engine service interface to set, initialize and start an application.\n[2024-11-18 13:43:13 +0000] [100] [INFO] Starting sync of cloud applications\n[INFO] [1731937393.151675196] [EventEngineInterface]: Successfully connected to Event Engine services\n[2024-11-18 13:43:13 +0000] [100] [INFO] Synced cloud applications: 0 added, 0 updated, 0 deleted, 1 total\n"})}),"\n",(0,t.jsxs)(n.admonition,{type:"info",children:[(0,t.jsx)(n.p,{children:"If there are any errors, check that the license file is valid and has been mounted correctly. For example, the following\nerror would be shown from a correctly mounted but invalid license file:"}),(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-console",children:"[ERROR] [1731937393.377201011] [licensing]: Error: license is invalid (ERR - license is invalid), please check that it is correct\n"})}),(0,t.jsx)(n.p,{children:"Contact AICA support if the container does not start correctly despite a valid license file."}),(0,t.jsx)(n.p,{children:"There can also be harmless warnings that appear if Cloud Storage is not set up or if the license verification takes\nlonger that a few seconds:"}),(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-console",children:"[2024-11-18 13:38:16 +0000] [135] [INFO] Starting sync of cloud applications\n[2024-11-18 13:38:16 +0000] [135] [WARNING] Sync failed\n[2024-11-18 13:08:42 +0000] [151] [INFO] Waiting for licensing status... 5\n[WARN] [1731935323.407252919] [EventEngine.ServiceHandler]: (404): Could not determine any license status\n"})})]}),"\n",(0,t.jsx)(n.h2,{id:"stopping-the-application-container",children:"Stopping the application container"}),"\n",(0,t.jsxs)(n.p,{children:["To shut down the AICA application container at any time, press CTRL+C in the original terminal window. Alternatively,\nto stop the application container from a different terminal window, look up the container name\nwith ",(0,t.jsx)(n.code,{children:"docker container ps"})," and then run ",(0,t.jsx)(n.code,{children:"docker container stop <container_name>"}),"."]}),"\n",(0,t.jsx)(n.h3,{id:"persistent-user-data",children:"Persistent user data"}),"\n",(0,t.jsxs)(n.p,{children:["AICA applications, URDF hardware and user configurations managed through the API or AICA Studio are stored in a\ndatabase. Because the docker container is isolated from the host filesystem, the local database will be lost when the\ncontainer exits. To persist local data between sessions, create a dedicated directory somewhere on the host. For\nexample, use ",(0,t.jsx)(n.code,{children:"mkdir ~/.aica-data"})," to keep the data folder hidden in the home folder. Then execute the normal run command\nwith an additional volume mount for the user data."]}),"\n",(0,t.jsx)(n.admonition,{type:"note",children:(0,t.jsxs)(n.p,{children:["Change ",(0,t.jsx)(n.code,{children:"/path/to/data"})," in the command below to a desired location for the data (e.g., ",(0,t.jsx)(n.code,{children:"~/.aica-data"})," or elsewhere)"]})}),"\n",(0,t.jsxs)(s.A,{groupId:"os",children:[(0,t.jsx)(r.A,{value:"linux",label:"Linux",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"docker run -it --rm \\\n  --privileged \\\n  --net=host \\\n  -v /path/to/license:/license:ro \\\n  #highlight-next-line\n  -v /path/to/data:/data:rw \\\n  aica-runtime\n"})})}),(0,t.jsx)(r.A,{value:"mac",label:"macOS",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"docker run -it --rm \\\n  --privileged \\\n  -p 8080:8080 -p 18000-18100:18000-18100/udp \\\n  -v /path/to/license:/license:ro \\\n  #highlight-next-line\n  -v /path/to/data:/data:rw \\\n  aica-runtime\n"})})})]}),"\n",(0,t.jsx)(n.h3,{id:"setting-a-super-admin-password",children:"Setting a super-admin password"}),"\n",(0,t.jsx)(n.p,{children:"AICA Core v4.3.0 and later require authentication for AICA Studio and the API. If no users exist in the mounted user\ndatabase, as is the case for a new configuration or when no data folder is mounted, a system administration password can\nbe set through an environment variable which grants full administration rights."}),"\n",(0,t.jsxs)(s.A,{groupId:"os",children:[(0,t.jsx)(r.A,{value:"linux",label:"Linux",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:'docker run -it --rm \\\n  --privileged \\\n  --net=host \\\n  -v /path/to/license:/license:ro \\\n  #highlight-next-line\n  -e AICA_SUPER_ADMIN_PASSWORD="${AICA_SUPER_ADMIN_PASSWORD}" \\\n  aica-runtime\n'})})}),(0,t.jsx)(r.A,{value:"mac",label:"macOS",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:'docker run -it --rm \\\n  --privileged \\\n  -p 8080:8080 -p 18000-18100:18000-18100/udp \\\n  -v /path/to/license:/license:ro \\\n  #highlight-next-line\n  -e AICA_SUPER_ADMIN_PASSWORD="${AICA_SUPER_ADMIN_PASSWORD}" \\\n  aica-runtime\n'})})})]}),"\n",(0,t.jsx)(n.h2,{id:"access-the-aica-studio",children:"Access the AICA Studio"}),"\n",(0,t.jsxs)(n.p,{children:["Visit ",(0,t.jsx)(n.a,{href:"http://localhost:8080",children:"localhost:8080"})," in the browser while the container is running to view AICA Studio. If the\n",(0,t.jsx)(n.code,{children:"AICA_SUPER_ADMIN_PASSWORD"})," environment variable was set, use the ",(0,t.jsx)(n.code,{children:"super-admin"})," username and the provided password to\nlog in as the system administrator."]}),"\n",(0,t.jsx)(n.h2,{id:"access-the-rest-api",children:"Access the REST API"}),"\n",(0,t.jsxs)(n.p,{children:["Visit ",(0,t.jsx)(n.a,{href:"http://localhost:8080/api",children:"localhost:8080/api"})," to see the Swagger UI and documentation for the REST API."]}),"\n",(0,t.jsx)(n.h2,{id:"connect-a-terminal-session-to-the-container",children:"Connect a terminal session to the container"}),"\n",(0,t.jsxs)(n.p,{children:["It is sometimes useful to connect to the application container while it is running to inspect files or run commands.\nThis can be accomplished with the ",(0,t.jsx)(n.code,{children:"docker container exec"})," command using the container name."]}),"\n",(0,t.jsx)(n.admonition,{type:"note",children:(0,t.jsxs)(n.p,{children:["Change ",(0,t.jsx)(n.code,{children:"CONTAINER_NAME"})," in the command below to the name of the running container."]})}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"docker container exec -it -u ros2 CONTAINER_NAME /bin/bash\n"})}),"\n",(0,t.jsxs)(n.p,{children:["The flags ",(0,t.jsx)(n.code,{children:"-it -u ros2"})," tell docker to attach an interactive terminal as the ",(0,t.jsx)(n.code,{children:"ros2"})," user. The command ",(0,t.jsx)(n.code,{children:"/bin/bash"}),"\nstarts a shell process."]}),"\n",(0,t.jsxs)(n.admonition,{type:"tip",children:[(0,t.jsxs)(n.p,{children:["You can find the names of all running containers with the ",(0,t.jsx)(n.code,{children:"docker ps"})," command."]}),(0,t.jsxs)(n.p,{children:["Docker containers can also be given an explicit name when started with ",(0,t.jsx)(n.code,{children:"docker run --name CONTAINER_NAME [...]"}),"."]})]}),"\n",(0,t.jsxs)(n.p,{children:["Once attached, run any commands in the context of the container. For example, ",(0,t.jsx)(n.code,{children:"ros2 topic list"})," will show the current\nROS 2 topics."]}),"\n",(0,t.jsxs)(n.p,{children:["Detach from the container with CTRL+D or with the ",(0,t.jsx)(n.code,{children:"exit"})," command."]}),"\n",(0,t.jsx)(n.h3,{id:"display-sharing",children:"Display sharing"}),"\n",(0,t.jsx)(n.admonition,{type:"note",children:(0,t.jsx)(n.p,{children:"Native display sharing for GUI applications is only supported on Linux hosts."})}),"\n",(0,t.jsx)(n.p,{children:"To run GUI applications like RViz in the docker container, it is necessary to share additional environment variables\nfor the display and X11 server."}),"\n",(0,t.jsxs)(n.admonition,{type:"caution",children:[(0,t.jsxs)(n.p,{children:["Running ",(0,t.jsx)(n.code,{children:"xhost +"})," disables access control to the X11 server on the host, allowing the container to access the display.\nIt also allows any other process to access the server, which carries security implications. Use with caution."]}),(0,t.jsxs)(n.p,{children:["Access control can be re-enabled with ",(0,t.jsx)(n.code,{children:"xhost -"}),"."]})]}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:'xhost +\ndocker container exec -it -u ros2 -e DISPLAY="$DISPLAY" -e XAUTHORITY="$XAUTH" CONTAINER_NAME /bin/bash\n'})}),"\n",(0,t.jsxs)(n.p,{children:["You should then be able to run ",(0,t.jsx)(n.code,{children:"rviz2"})," inside the container and see the window appear."]})]})}function p(e={}){const{wrapper:n}={...(0,o.R)(),...e.components};return n?(0,t.jsx)(n,{...e,children:(0,t.jsx)(u,{...e})}):u(e)}},5537:(e,n,a)=>{a.d(n,{A:()=>y});var i=a(6540),t=a(8215),o=a(5627),s=a(6347),r=a(372),c=a(604),l=a(1861),d=a(8749);function h(e){return i.Children.toArray(e).filter((e=>"\n"!==e)).map((e=>{if(!e||(0,i.isValidElement)(e)&&function(e){const{props:n}=e;return!!n&&"object"==typeof n&&"value"in n}(e))return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))?.filter(Boolean)??[]}function u(e){const{values:n,children:a}=e;return(0,i.useMemo)((()=>{const e=n??function(e){return h(e).map((e=>{let{props:{value:n,label:a,attributes:i,default:t}}=e;return{value:n,label:a,attributes:i,default:t}}))}(a);return function(e){const n=(0,l.XI)(e,((e,n)=>e.value===n.value));if(n.length>0)throw new Error(`Docusaurus error: Duplicate values "${n.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[n,a])}function p(e){let{value:n,tabValues:a}=e;return a.some((e=>e.value===n))}function m(e){let{queryString:n=!1,groupId:a}=e;const t=(0,s.W6)(),o=function(e){let{queryString:n=!1,groupId:a}=e;if("string"==typeof n)return n;if(!1===n)return null;if(!0===n&&!a)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return a??null}({queryString:n,groupId:a});return[(0,c.aZ)(o),(0,i.useCallback)((e=>{if(!o)return;const n=new URLSearchParams(t.location.search);n.set(o,e),t.replace({...t.location,search:n.toString()})}),[o,t])]}function g(e){const{defaultValue:n,queryString:a=!1,groupId:t}=e,o=u(e),[s,c]=(0,i.useState)((()=>function(e){let{defaultValue:n,tabValues:a}=e;if(0===a.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(n){if(!p({value:n,tabValues:a}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${n}" but none of its children has the corresponding value. Available values are: ${a.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return n}const i=a.find((e=>e.default))??a[0];if(!i)throw new Error("Unexpected error: 0 tabValues");return i.value}({defaultValue:n,tabValues:o}))),[l,h]=m({queryString:a,groupId:t}),[g,x]=function(e){let{groupId:n}=e;const a=function(e){return e?`docusaurus.tab.${e}`:null}(n),[t,o]=(0,d.Dv)(a);return[t,(0,i.useCallback)((e=>{a&&o.set(e)}),[a,o])]}({groupId:t}),f=(()=>{const e=l??g;return p({value:e,tabValues:o})?e:null})();(0,r.A)((()=>{f&&c(f)}),[f]);return{selectedValue:s,selectValue:(0,i.useCallback)((e=>{if(!p({value:e,tabValues:o}))throw new Error(`Can't select invalid tab value=${e}`);c(e),h(e),x(e)}),[h,x,o]),tabValues:o}}var x=a(9136);const f={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};var j=a(4848);function v(e){let{className:n,block:a,selectedValue:i,selectValue:s,tabValues:r}=e;const c=[],{blockElementScrollPositionUntilNextRender:l}=(0,o.a_)(),d=e=>{const n=e.currentTarget,a=c.indexOf(n),t=r[a].value;t!==i&&(l(n),s(t))},h=e=>{let n=null;switch(e.key){case"Enter":d(e);break;case"ArrowRight":{const a=c.indexOf(e.currentTarget)+1;n=c[a]??c[0];break}case"ArrowLeft":{const a=c.indexOf(e.currentTarget)-1;n=c[a]??c[c.length-1];break}}n?.focus()};return(0,j.jsx)("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,t.A)("tabs",{"tabs--block":a},n),children:r.map((e=>{let{value:n,label:a,attributes:o}=e;return(0,j.jsx)("li",{role:"tab",tabIndex:i===n?0:-1,"aria-selected":i===n,ref:e=>{c.push(e)},onKeyDown:h,onClick:d,...o,className:(0,t.A)("tabs__item",f.tabItem,o?.className,{"tabs__item--active":i===n}),children:a??n},n)}))})}function b(e){let{lazy:n,children:a,selectedValue:o}=e;const s=(Array.isArray(a)?a:[a]).filter(Boolean);if(n){const e=s.find((e=>e.props.value===o));return e?(0,i.cloneElement)(e,{className:(0,t.A)("margin-top--md",e.props.className)}):null}return(0,j.jsx)("div",{className:"margin-top--md",children:s.map(((e,n)=>(0,i.cloneElement)(e,{key:n,hidden:e.props.value!==o})))})}function k(e){const n=g(e);return(0,j.jsxs)("div",{className:(0,t.A)("tabs-container",f.tabList),children:[(0,j.jsx)(v,{...n,...e}),(0,j.jsx)(b,{...n,...e})]})}function y(e){const n=(0,x.A)();return(0,j.jsx)(k,{...e,children:h(e.children)},String(n))}},8453:(e,n,a)=>{a.d(n,{R:()=>s,x:()=>r});var i=a(6540);const t={},o=i.createContext(t);function s(e){const n=i.useContext(o);return i.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function r(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(t):e.components||t:s(e.components),i.createElement(o.Provider,{value:n},e.children)}},9329:(e,n,a)=>{a.d(n,{A:()=>s});a(6540);var i=a(8215);const t={tabItem:"tabItem_Ymn6"};var o=a(4848);function s(e){let{children:n,hidden:a,className:s}=e;return(0,o.jsx)("div",{role:"tabpanel",className:(0,i.A)(t.tabItem,s),hidden:a,children:n})}}}]);