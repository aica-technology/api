all:
.PHONY: all

generate:
	uv run openapi-python-client generate --overwrite --url http://localhost:8080/api/schema.yaml --output-path src/aica_api/sdk --config .openapi.config.yaml
.PHONY: generate

vet:
	uv tool run ruff check
	uv tool run ruff format --diff
.PHONY: vet

build: generate
	uv build
.PHONY: build

example:
	uv run examples/with_api_key.py
.PHONY: example
