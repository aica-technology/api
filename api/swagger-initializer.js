const PythonRequestsPlugin = {
	fn: {
		requestSnippetGenerator_python_requests: (request) => {
			const url = request.get("url");
			const method = request.get("method").toLowerCase();
			const body = request.get("body");
			const headers = request
				.get("headers")
				.map((value, key) => `    "${key}": "${value}"`)
				.join(",\n");

			let data = "";
			if (body) {
				try {
					const parsedBody = JSON.parse(body);
					const json = Object.entries(parsedBody)
						.map(([key, value]) => `    "${key}": ${JSON.stringify(value)}`)
						.join(",\n");
					data = `  json={\n${json}\n  },\n`;
				} catch (e) {
					data = `  data="${body}",\n`;
				}
			}

			return `import requests

response = requests.${method}(
  "${url}",
  headers={
${headers}
  },
${data})

print(response.text)`;
		},
	},
};

window.onload = () => {
	const baseURL = `${window.location.protocol}//${window.location.host}${window.location.pathname}`;
	window.ui = SwaggerUIBundle({
		url: `${baseURL}schema.yaml`,
		dom_id: "#swagger-ui",
		deepLinking: true,
		filter: true,
		presets: [SwaggerUIBundle.presets.apis, SwaggerUIStandalonePreset],
		plugins: [PythonRequestsPlugin],
		layout: "BaseLayout",
		responseInterceptor: (response) => {
			response.text = response.text.replace(/__AICA_LOCAL_SERVER__/g, baseURL);
			return response;
		},
		requestSnippetsEnabled: true,
		requestSnippets: {
			generators: {
				python_requests: {
					title: "Python (requests)",
					syntax: "python",
				},
			},
			languages: ["curl_bash", "python_requests"],
		},
	});
};
