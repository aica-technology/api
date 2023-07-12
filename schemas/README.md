# Schemas

This directory contains JSON schemas to define public structures and syntax for AICA applications and components

## Application schema

The [applications](./applications) directory defines the YAML syntax for AICA applications.

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