FROM python:3.9-alpine AS dependencies

RUN apk update && apk add gcc libc-dev
RUN python -m pip install check-jsonschema json-schema-for-humans


FROM dependencies AS sources
WORKDIR /
COPY ./schema /schema


FROM sources AS generate-docs
RUN mkdir -p /html && generate-schema-doc /schema /html


FROM generate-docs AS serve-html
WORKDIR /html
CMD python -m http.server


FROM sources AS validate

ENTRYPOINT ["check-jsonschema", "--schemafile"]
