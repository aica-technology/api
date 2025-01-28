import os

from aica_api.client import AICA

client = AICA(
    api_key=os.getenv('AICA_API_KEY'),
)

print(f"Check: {"pass" if client.check() else "failed"}")
print(f'Core Version: {client.core_version()}')
print(f'Protocol: {client.protocol()}')
print(f'Application state: {client.get_application_state().text}')
print(f'Load component: {client.load_component("def").text}')
print(f'Wait for component: {client.wait_for_component('abc', 'loaded', 5)}')
