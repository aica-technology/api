# AICA API Resources

This repository contains tools and resources for interacting with the AICA API, applications and components.

## Python client

The Python client is available at https://pypi.org/project/aica-api/ and can be installed as follows:
```shell
pip install aica-api
```

See the [python](./python) subdirectory for more information and source code.

## Schemas

JSON schemas defining the expected syntax for AICA application files or component descriptions are available
in the [schemas](./schemas) subdirectory.

### Bundling the schemas locally

In order to bundle the schemas locally for debugging and testing, do the following (replace `<path>` and `<schema>` for the
desired schema generation):

```bash
docker build -t aica-technology/api-schema - <<EOF
FROM node:latest
WORKDIR /tmp
COPY . .
RUN npm install utils
RUN node utils/bundleHelper.js <path>/<schema>.schema.json <schema>.schema.json
EOF
CONTAINER_ID=$(docker run -d aica-technology/api-schema)
docker cp "${CONTAINER_ID}":/tmp/<schema>.schema.json .
docker stop "${CONTAINER_ID}"
```