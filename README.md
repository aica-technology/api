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

In order to bundle the schemas locally for debugging and testing, do the following (replace `SCHEMA_NAME` and
`SCHEMA_PATH` with the appropriate values for other schemas):

```bash
SCHEMA_NAME="interfaces"
SCHEMA_PATH="schemas/interfaces/schema"
docker build -t aica-technology/api-schema -f- . <<EOF
FROM node:latest
WORKDIR /tmp
COPY . .
RUN cd utils && npm install
RUN node utils/bundleHelper.js "${SCHEMA_PATH}"/"${SCHEMA_NAME}".schema.json "${SCHEMA_NAME}"
EOF
if [ $? -eq 0 ]; then \
  CONTAINER_ID=$(docker run -d aica-technology/api-schema);
  docker cp "${CONTAINER_ID}":/tmp/"${SCHEMA_NAME}".schema.json .;
  docker cp "${CONTAINER_ID}":/tmp/"${SCHEMA_NAME}".types.schema.json .;
  docker stop "${CONTAINER_ID}";
  docker rm "${CONTAINER_ID}";
fi
```