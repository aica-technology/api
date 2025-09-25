---
sidebar_position: 1
title: Creating a custom package from templates
---

# Creating a package

Custom components or controllers can be defined in ROS 2 packages and used alongside the standard library of AICA
components and controllers in AICA Core. **Components** can be implemented in C++ or Python. A component package can
contain multiple components in either language, while a **controller** package can only be implemented in C++. On top of
that, multiple component and controller packages can be organized under the same directory and described by a single
`aica-package.toml`. In this case, we refer to them as a **collection of packages** or simply a **collection**.

The following sections describe how you can write a [component](./components/component-package.md) and a
[controller](./controllers/controller-package.md), as well as, how to create a collection out of multiple packages.