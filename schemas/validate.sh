#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: ./validate.sh /path/to/application.yaml"
    exit 0
fi

YAML_FILE="$1"
FULL_PATH="$(cd "$(dirname "$1")" || exit 1; pwd)"
FILENAME=$(basename "${YAML_FILE}")

IMAGE_NAME=aica-technology/api/application-schema:validate

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
docker build --target validate --tag "${IMAGE_NAME}" "${SCRIPT_DIR}" || exit 1

# mount a volume to share the file, attach to stderr, pass the filename to the entrypoint and redirect any output from stderr to stdout
STDERR_CAPTURE=$(docker run -a stderr --rm --volume "$FULL_PATH":/home/validate "${IMAGE_NAME}" \
  "/home/validate/${FILENAME}" 2>&1 >/dev/null)
if [ -n "$STDERR_CAPTURE" ]; then
  echo "Failure: ${FILENAME} is not a valid application description!"
  echo "${STDERR_CAPTURE}"
else
  echo "Success: ${FILENAME} is a valid application description."
fi
