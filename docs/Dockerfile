FROM node:lts

WORKDIR /docs
COPY . .
RUN --mount=type=cache,target=/root/.npm npm install
RUN --mount=type=cache,target=/docs/.docusaurus npm run build
CMD npm run start:docker
