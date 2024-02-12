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

The latest version of the AICA API client will generally support the latest API version in the AICA framework. For
older versions of the AICA framework, it may be necessary to install older versions of the client. Use the following
compatability table to determine which client version to use.

| API server version | Matching Python client version |
|--------------------|--------------------------------|
| `v3.x`             | `>= v2.0.0`                    |
| `v2.x`             | `v1.2.0`                       |
| `<= v1.x`          | Unsupported                    |

Recent client versions also support the following functions to check the client version and API compatability.

```python
from aica_api.client import AICA

aica = AICA()

# get the current client version
aica.client_version()

# check compatability between the client version and API version
aica.check()
```