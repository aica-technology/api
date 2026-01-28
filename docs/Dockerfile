FROM oven/bun:1.3

WORKDIR /docs
COPY . .
RUN --mount=type=cache,target=/root/.bun bun install
RUN --mount=type=cache,target=/docs/.docusaurus bun run build
CMD ["bun", "run", "start:docker"]
