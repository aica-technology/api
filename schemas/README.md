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

## Extensions

The [extensions](./extensions) unifies the previous description schemas for components and controllers.

## Interfaces

The [interfaces](./interfaces) directory defines a schema with common interface definitions used by both
the component and controller descriptions.

## Bundling the schemas

In order to bundle the schemas for debugging and testing, do the following (replace `SCHEMA_NAME` and `SCHEMA_PATH`
with the appropriate values for other schemas):

```bash
SCHEMA_NAME="interfaces"
SCHEMA_PATH="interfaces/schema"
bun install
bun run bundleHelper.js "${SCHEMA_PATH}"/"${SCHEMA_NAME}".schema.json "${SCHEMA_NAME}"
```

or, with Docker:

```bash
SCHEMA_NAME="interfaces"
SCHEMA_PATH="interfaces/schema"
docker build -t aica-technology/api-schema -f- . <<EOF
FROM oven/bun:1.3
WORKDIR /tmp
COPY . .
RUN bun install
RUN bun run bundleHelper.js "${SCHEMA_PATH}"/"${SCHEMA_NAME}".schema.json "${SCHEMA_NAME}"
EOF
if [ $? -eq 0 ]; then \
  CONTAINER_ID=$(docker run -d aica-technology/api-schema);
  docker cp "${CONTAINER_ID}":/tmp/"${SCHEMA_NAME}".schema.json .;
  docker cp "${CONTAINER_ID}":/tmp/"${SCHEMA_NAME}".types.schema.json .;
  docker stop "${CONTAINER_ID}";
  docker rm "${CONTAINER_ID}";
fi
```

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
- `extensions`
- `interfaces`
