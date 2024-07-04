# Python AICA API Client

The AICA API client module provides simple functions for interacting with the AICA API.

```shell
pip install aica-api
```

The client can be used to easily make API calls as shown below:

```python
from aica_api.client import AICA

aica = AICA()

aica.set_application('my_application.yaml')
aica.start_application()

aica.load_component('my_component')
aica.unload_component('my_component')

aica.stop_application()
```

To check the status of component predicates and conditions, the following blocking methods can be employed:

```python
from aica_api.client import AICA

aica = AICA()

if aica.wait_for_condition('timer_1_active', timeout=10.0):
    print('Condition is true!')
else:
    print('Timed out before condition was true')

if aica.wait_for_predicate('timer_1', 'is_timed_out', timeout=10.0):
    print('Predicate is true!')
else:
    print('Timed out before predicate was true')
```

## Compatability table

The latest version of the AICA API client will generally support the latest API server version in the AICA framework.
Major changes to the API client or server versions indicate breaking changes and are not backwards compatible. To
interact with older versions of the AICA framework, it may be necessary to install older versions of the client.
Use the following compatability table to determine which client version to use.

| API server version | Matching Python client version  |
|--------------------|---------------------------------|
| `3.x`              | `>= 2.0.0`                      |
| `2.x`              | `1.2.0`                         |
| `<= 1.x`           | Unsupported                     |

Between major version changes, minor updates to the API server version and Python client versions may introduce new
endpoints and functions respectively. If a function requires a feature that the detected API server version does not yet
support (as is the case when the Python client version is more up-to-date than the targeted API server), then calling
that function will return None with a warning.

Recent client versions also support the following functions to check the client version and API compatability.

```python
from aica_api.client import AICA

aica = AICA()

# get the current API server version
print(aica.api_version())
# get the current client version
print(aica.client_version())

# check compatability between the client version and API version
if aica.check():
    print('Versions are compatible')
else:
    print('Versions are incompatible')
```
