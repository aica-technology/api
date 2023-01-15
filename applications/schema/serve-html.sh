#!/bin/bash

if [ "$#" -ne 0 ]; then
    echo "Usage: ./serve-html.sh"
    exit 0
fi

IMAGE_NAME=aica-technology/api/application-schema:html
CONTAINER_NAME=aica-technology-api-application-schema-html

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
docker build --target serve-html --tag "${IMAGE_NAME}" "${SCRIPT_DIR}"

echo "Starting an html server container on localhost:8000"
docker run -d --rm -p 8000:8000 --name "${CONTAINER_NAME}" "${IMAGE_NAME}"

WEB_PATH=http://localhost:8000/application.schema.html
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