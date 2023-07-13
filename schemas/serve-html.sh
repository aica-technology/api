#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "View JSON schema as readable HTML in the browser"
  echo "Usage: ./serve-html.sh <schema_collection>"
  echo
  echo "Currently supported schema collections:"
  echo "  - applications"
  echo "  - component-descriptions"
  echo
  exit 0
fi

SCHEMA="$1"
SCHEMA_HTML=""
if [ "$SCHEMA" == "applications" ]; then
  echo "Using AICA application schema"
  SCHEMA_HTML="application.schema.html"
elif [ "$SCHEMA" == "component-descriptions" ]; then
  echo "Using AICA component description schema"
  SCHEMA_HTML="component.schema.html"
elif [ "$SCHEMA" == "controller-descriptions" ]; then
  echo "Using AICA controller description schema"
  SCHEMA_HTML="controller.schema.html"
elif [ "$SCHEMA" == "parameters" ]; then
  echo "Using AICA parameter schema"
  SCHEMA_HTML="parameter.schema.html"
elif [ "$SCHEMA" == "signals" ]; then
  echo "Using AICA signal schema"
  SCHEMA_HTML="signal.schema.html"
else
  echo "Invalid schema option: $SCHEMA"
  exit 0
fi

IMAGE_NAME="aica-technology/api/${SCHEMA}-schema:html"
CONTAINER_NAME="aica-technology-api-${SCHEMA}-schema-html"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
docker build -f Dockerfile --target serve-html --tag "${IMAGE_NAME}" "${SCRIPT_DIR}/${SCHEMA}" || exit 1

echo "Starting an html server container on localhost:8000"
docker run -d --rm -p 8000:8000 --name "${CONTAINER_NAME}" "${IMAGE_NAME}"

WEB_PATH="http://localhost:8000/${SCHEMA_HTML}"
if [ "$(command -v python)" ]; then
  python -m webbrowser "${WEB_PATH}"
elif [ "$(command -v python3)" ]; then
  python3 -m webbrowser "${WEB_PATH}"
elif [ "$(command -v php)" ]; then
  echo "HTML server running! Visit the following address in a browser:"
  echo ">> ${WEB_PATH}"
fi

read -n 1 -s -r -p "Press any key to stop the html server container"
echo ""
echo "Stopping the container"
docker container rm --force "${CONTAINER_NAME}"
