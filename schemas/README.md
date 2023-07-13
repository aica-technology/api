# Schemas

This directory contains JSON schemas to define public structures and syntax for AICA applications and components

## Application schema

The [applications](./applications) directory defines the YAML syntax for AICA applications.

## Component Description schema

The [component-descriptions](./component-descriptions) directory defines the JSON syntax for describing the properties
of AICA component classes.

## Controller Description schema

The [controller-descriptions](./controller-descriptions) directory defines the JSON syntax for describing the properties
of AICA controller plugins.

## Parameters and Signals schema

The [parameters](./parameters) and [signals](./signals) directories contains common schema definitions used by both
the component and controller descriptions.

## Tools

The Dockerfile and bash scripts can be used to easily view or validate JSON schemas.

Run `./serve-html.sh <schema_collection>` to render a chosen JSON schema group as human-readable HTML.
For example, to view the application schema:
```shell
./serve-html.sh applications
```

Run `./validate.sh <schema_collection> <file>` to validate an instance file (JSON or YAML) against a chosen JSON schema.
For example, to validate some YAML application file:
```shell
./validate.sh applications ../path/to/example_application.yaml
```

Available `schema_collection` options are:
- `applications`
- `component-descriptions`
- `controller-descriptions`
- `parameters`
- `signals`
