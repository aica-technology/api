import os

from aica_api.client import AICA

client = AICA(
    api_key=os.getenv('AICA_API_KEY'),
)

assert client.check()
print(f'Application state: {client.get_application_state().text}')
print(client.wait_for_component('abc', 'on'))
