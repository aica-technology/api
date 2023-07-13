FROM python:3.9-alpine as dependencies

RUN apk update && apk add gcc libc-dev
RUN python -m pip install check-jsonschema json-schema-for-humans


FROM dependencies as sources
WORKDIR /
COPY ./schema /schema


FROM sources as generate-docs
RUN mkdir -p /html && generate-schema-doc /schema /html


FROM generate-docs as serve-html
WORKDIR /html
CMD python -m http.server


FROM sources as validate

ENTRYPOINT ["check-jsonschema", "--schemafile"]
