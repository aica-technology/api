#!/bin/bash

set -e

BASE_TAG="latest"
LICENSE_FILE="./license"
PACKAGES=()
FINAL_TAG="aica-technology/app:${BASE_TAG}"
REGISTRY="registry.licensing.aica.tech"
LOCAL_IMAGES=no
BASE_IMAGE="base"

usage() {
  if [ -n "$1" ]; then
    echo "Error: $1"
  fi
  echo "Usage: $0 [options]"
  echo "Options:"
  echo "  --base <version>               base image version"
  echo "  --packages <pkg1> <pkg2>       packages to install (e.g. 'collections/ur' or 'components/gear-assembly')"
  echo "  --license <file>               license file to use to connect to the registry (default: '${LICENSE_FILE}')"
  echo "  --tag <tag>                    docker tag to use for the final image"
  echo "  -h, --help                     display this help"
  if [ -n "${DEBUG_CLI}" ]; then
    echo "Debugging options:"
    echo "  --registry <registry>          registry to use (default: '${REGISTRY}')"
    echo "  --local-images                 use local images instead of the registry"
    echo "  --base-image <tag>             override the base image name and tag (requires --local-images)"
    echo "  --                             stop parsing options, pass the rest to Docker build"
  fi
  exit 1
}

enforce_args() {
  if [ "$#" -lt "$1" ]; then
    NUM=$(( $1 - $# ))
    usage "missing ${NUM} argument(s) for $2"
    exit 1
  fi
}

make_image_name() {
  local image_with_tag="$1"
  local image="$2"
  if [ "${LOCAL_IMAGES}" = "yes" ]; then
    echo "${image}"
  else
    echo "${REGISTRY}/${image_with_tag}"
  fi
}

main() {
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --base) enforce_args 2 "$1"; BASE_TAG="$2"; shift 2;;
      --license) enforce_args 2 "$1"; LICENSE_FILE="$2"; shift 2;;
      --tag) enforce_args 2 "$1"; FINAL_TAG="$2"; shift 2;;
      --registry) enforce_args 2 "$1"; REGISTRY="$2"; shift 2;;
      --local-images) LOCAL_IMAGES=yes; shift;;
      --base-image) enforce_args 2 "$1"; BASE_IMAGE="$2"; shift 2;;
      -h|--help) usage; shift;;
      --packages)
        enforce_args 2 "$1"
        shift
        # until we run into something that looks like an option
        while [ "$#" -gt 0 ]; do
          case "$1" in
            -*|--*) break;;
          esac
          PACKAGES+=("$1")
          shift
        done
        ;;
      --) shift; break;;
      *) break;;
    esac
  done

  echo "# Checking arguments"
  if [ "${LOCAL_IMAGES}" = "no" ]; then
    if [ -z "${BASE_TAG}" ]; then
      usage "missing --base <version>"
    fi
  fi
  base_image="$(make_image_name "${BASE_IMAGE}:${BASE_TAG}" "${BASE_IMAGE}")"

  if [ ! -f "${LICENSE_FILE}" ]; then
    usage "license file '${LICENSE_FILE}' does not exist"
  fi

  echo "# Logging to the registry with your license"
  cat "${LICENSE_FILE}" | docker login "${REGISTRY}" --username package.sh --password-stdin

  echo "# Building your image"
  for package in "${PACKAGES[@]}"; do
    package_image="$(make_image_name "${package}" "${package}")"
    uname="${package//[:.\/]/_}"
    steps_FROM="${steps_FROM}FROM ${package_image} AS ${uname}
"
    steps_RUN="${steps_RUN}COPY --from=${uname} /colcon \${WORKSPACE}/install
"
  done

  docker build - -t ${FINAL_TAG} ${@} <<EOF
#syntax=docker/dockerfile:1.4.0
${steps_FROM}
FROM ${base_image}
${steps_RUN}
EOF

  echo "# Finished"
  echo "You can now run your image with:"
  LICENSE_FILE_PATH="$(realpath "${LICENSE_FILE}")"
  echo "  docker run --privileged --rm -v'${LICENSE_FILE_PATH}':/license:ro ${FINAL_TAG}"
}

main "$@"
