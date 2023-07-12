#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Validate a JSON or YAML file against a JSON schema"
  echo "Usage: ./validate.sh <schema_collection> <path_to_file>"
  echo
  echo "Currently supported schema collections:"
  echo "  - applications"
  echo "  - component-descriptions"
  echo
  exit 0
fi

SCHEMA="$1"
SCHEMA_ENTRYPOINT=""
if [ "$SCHEMA" == "applications" ]; then
  echo "AICA application schema"
  SCHEMA_ENTRYPOINT="application.schema.json"
elif [ "$SCHEMA" == "component-descriptions" ]; then
  echo "Using AICA component description schema"
  SCHEMA_ENTRYPOINT="component.schema.json"
else
  echo "Invalid schema option: $SCHEMA"
  exit 0
fi

FIXTURE_FILE="$2"
FULL_PATH="$(
  cd "$(dirname "$2")" || exit 1
  pwd
)"
FILENAME=$(basename "${FIXTURE_FILE}")

IMAGE_NAME="aica-technology/api/${SCHEMA}-schema:validate"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
docker build -f Dockerfile --target validate --tag "${IMAGE_NAME}" "${SCRIPT_DIR}/${SCHEMA}" || exit 1

echo "${FULL_PATH}"

# mount a volume to share the fixture file and pass the relevant paths as command arguments to the entrypoint
RESULT=$(
  docker run --rm \
    --volume "${FULL_PATH}":/home/validate \
    "${IMAGE_NAME}" \
    "/schema/${SCHEMA_ENTRYPOINT}" "/home/validate/${FILENAME}"
)

if [ "$RESULT" == "ok -- validation done" ]; then
  echo "Success: ${FILENAME} is a valid according to the ${SCHEMA} schema"
else
  echo "Failure: ${FILENAME} is not valid according to the ${SCHEMA} schema!"
  echo "${RESULT}"
fi
