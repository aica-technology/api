import json
import logging
import os

from aica_api.client import AICA

api_key = os.getenv('AICA_API_KEY')
if not api_key:
    raise ValueError('AICA_API_KEY environment variable is not set')
client = AICA(api_key=api_key, url=os.getenv('AICA_API_URL', 'http://localhost:8080/api'))

print(f'Check: {"pass" if client.check() else "failed"}')
print(f'Core Version: {client.core_version()}')
print(f'Protocol: {client.protocol()}')
print(f'Application state: {client.get_application_state()}')
try:
    print(f'Load component: {client.load_component("def")}')
except:  # noqa: E722
    logging.exception('load component failed')
client.set_application(json.dumps({'schema': '2-0-6', 'dependencies': {'core': 'v5.0.0'}}))
print(f'Application state: {client.get_application_state()}')
try:
    client.load_application('New Application')
except ValueError:
    print('Failed to load application by name')
print(f'Wait for component: {client.wait_for_component("abc", "loaded", 5)}')
