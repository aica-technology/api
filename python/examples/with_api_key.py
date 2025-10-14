import json
import os

from aica_api.client import AICA

client = AICA(
    api_key=os.getenv('AICA_API_KEY', ''),
)

print(f'Check: {"pass" if client.check() else "failed"}')
print(f'Core Version: {client.core_version()}')
print(f'Protocol: {client.protocol()}')
print(f'Application state: {client.get_application_state()}')
client.set_application(json.dumps({'schema': '2-0-6', 'dependencies': {'core': 'v5.0.0'}}))
print(f'Application state: {client.get_application_state()}')
try:
    client.load_application('New Application')
except ValueError:
    print('Failed to load application by name')
print(f'Wait for component: {client.wait_for_component("abc", "loaded", 5)}')
