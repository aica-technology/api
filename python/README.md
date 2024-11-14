# Python AICA API Client

The AICA API client module provides simple functions for interacting with the API of the AICA Core.

```shell
pip install aica-api
```

The client can be used to easily make API calls as shown below:

```python3
from aica_api.client import AICA

aica = AICA()

aica.set_application('my_application.yaml')
aica.start_application()

aica.load_component('my_component')
aica.unload_component('my_component')

aica.stop_application()
```

To check the status of predicates and conditions, the following blocking methods can be employed:

```python3
from aica_api.client import AICA

aica = AICA()

if aica.wait_for_condition('timer_1_active', timeout=10.0):
    print('Condition is true!')
else:
    print('Timed out before condition was true')

if aica.wait_for_component_predicate('timer_1', 'is_timed_out', timeout=10.0):
    print('Predicate is true!')
else:
    print('Timed out before predicate was true')
```

## Compatability table

The latest version of this AICA API client will generally support the latest AICA Core version.
Major version changes to the API client or to AICA Core indicate breaking changes and are not always backwards
compatible. To interact with older versions of AICA Core, it may be necessary to install older versions of the client.
Use the following compatability table to determine which client version to use.

| AICA Core version | API protocol version | Matching Python client version |
|-------------------|----------------------|--------------------------------|
| `4.x`             | `v2`                 | `>= 3.0.0`                     |
| `3.x`             | `v2`                 | `>= 2.0.0`                     |
| `2.x`             | `v2`                 | `1.2.0`                        |
| `<= 1.x`          | `v1`                 | Unsupported                    |

The API protocol version is a namespace for the endpoints. Endpoints under the `v2` protocol have a `/v2/...` prefix in
the URL. A change to the protocol version indicates a fundamental change to the API structure or behavior.

Between major version changes, minor updates to the AICA Core version and Python client versions may introduce new
endpoints and functions respectively. If a function requires a feature that the detected AICA Core version does not yet
support (as is the case when the Python client version is more up-to-date than the targeted AICA Core), then calling
that function will return None with a warning.

### Changes between major server versions

AICA Core versions `v1.x` and earlier were alpha and pre-alpha versions that are no longer supported.

AICA Core version `v2.x` was a beta version that introduced a new API structure under the `v2` protocol namespace.

In AICA Core `v3.x`, live data streaming for predicates and conditions switched from using raw websockets to Socket.IO
for data transfer. This constituted a breaking change to API clients, but the overall structure of the REST API remained
the same, and so the API protocol version is still `v2`.

AICA Core versions `v4.x` and later keep the same protocol structure as before under the `v2` namespace. The primary
breaking change from the point of the API server and client is that the `/version` endpoint now returns the version of
AICA Core, rather than the specific version of the API server subpackage inside core. These have historically carried
the same major version, but in future the core version may have major updates without any breaking changes to the
actual API server version.

### Checking compatibility

Recent client versions include a `check()` method to assess the client version and API compatability.

```python3
from aica_api.client import AICA

aica = AICA()

# check compatability between the client version and API version
if aica.check():
    print('Client and server versions are compatible')
else:
    print('Client and server versions are incompatible')
```

The latest client versions also include the following functions to check the configuration details manually.

```python3
from aica_api.client import AICA

aica = AICA()

# get the current version of this client
print(aica.client_version())

# get the current version of AICA Core (e.g. "4.0.0")
print(aica.core_version())

# get the current API protocol version (e.g. "v2")
print(aica.protocol())

# get the specific version of the API server running in AICA Core (e.g. "4.0.1")
# (generally only needed for debugging purposes)
print(aica.api_version())
```
