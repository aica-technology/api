# Python AICA API Client

```shell
pip install aica-api
```

The AICA API client module provides simple functions for interacting with the AICA Core through a REST API.

Refer to https://docs.aica.tech for more information about the AICA System.

## Basic usage

```python
from aica_api.client import AICA

aica = AICA()

if aica.check():
    print(f"Connected to AICA Core version {aica.core_version()}")
```

The client object can be used to easily make API calls to monitor or control AICA Core. For example:

```python3
aica.set_application('my_application.yaml')
aica.start_application()

aica.load_component('my_component')
aica.unload_component('my_component')

aica.stop_application()
```

To check the status of predicates and conditions, the following blocking methods can be employed:

```python3
if aica.wait_for_condition('timer_1_active', timeout=10.0):
    print('Condition is true!')
else:
    print('Timed out before condition was true')

if aica.wait_for_component_predicate('timer_1', 'is_timed_out', timeout=10.0):
    print('Predicate is true!')
else:
    print('Timed out before predicate was true')
```

Refer to the available methods of the `AICA` client class for more advanced usage.

## Network configuration

By default, the API server of AICA Core is available on the default address `localhost:8080`. Depending on the network
configuration, the URL or port number of the AICA Core instance may be different.

For example, when using AICA Launcher on macOS, the API is bound to a different, randomly generated port to avoid
conflict with reserved ports. Use the "Open in browser" button from Launcher to open AICA Studio in the browser and copy
the port from the url.

```python
from aica_api.client import AICA

# connect to a non-default port on the local network
aica = AICA(port=55000)

# or connect to a different host address entirely
aica = AICA(url='192.168.0.1', port=55005)
```

## Authentication with an API key

For connecting to AICA Core v4.3.0 and later, an API key is required for authentication.

API keys can be generated in AICA Studio with configurable access scopes. Note that available scopes are limited to
those of the currently logged-in user. A generated API key is only shown once and should be kept secret. For example, it
may be exported as an environment variable. The following example key is shown for demonstrative purposes only:

```shell
export AICA_API_KEY=64ce9e8f-aa46-4ba7-814f-f169c01c957e.RwoH6A1Ti5poNKSizoWrcBEYzh7AkB0kpMq1TR59t6os
```

The API key can then be provided to the constructor with the `api_key` keyword argument:

```python
import os
from aica_api.client import AICA

AICA_API_KEY = os.getenv('AICA_API_KEY')
aica = AICA(api_key=AICA_API_KEY)
```

## Compatability table

The latest version of this AICA API client will generally support the latest AICA Core version.
Major version changes to the API client or to AICA Core indicate breaking changes and are not always backwards
compatible. To interact with older versions of AICA Core, it may be necessary to install older versions of the client.
Use the following compatability table to determine which client version to use.

| AICA Core version | API protocol version | Matching Python client version |
|-------------------|----------------------|--------------------------------|
| `>= 4.3`          | `v2`                 | `>= 3.1.0`                     |
| `>= 4.0, < 4.3`   | `v2`                 | `>= 3.0.0`                     |
| `3.x`             | `v2`                 | `>= 2.0.0`                     |
| `2.x`             | `v2`                 | `1.2.0`                        |
| `<= 1.x`          | `v1`                 | Unsupported                    |

The API protocol version is a namespace for the endpoints. Endpoints under the `v2` protocol have a `/v2/...` prefix in
the URL. A change to the protocol version indicates a fundamental change to the API structure or behavior.

Between major version changes, minor updates to the AICA Core version and Python client versions may introduce new
endpoints and functions respectively. If a function requires a feature that the detected AICA Core version does not yet
support (as is the case when the Python client version is more up-to-date than the targeted AICA Core), then calling
that function will return None with a warning.

### Changes to API behavior between AICA Core versions

AICA Core versions `v1.x` and earlier were alpha and pre-alpha versions that are no longer supported.

AICA Core version `v2.x` was a beta version that introduced a new API structure under the `v2` protocol namespace.

In AICA Core `v3.x`, live data streaming for predicates and conditions switched from using raw websockets to Socket.IO
for data transfer. This constituted a breaking change to API clients, but the overall structure of the REST API remained
the same, and so the API protocol version is still `v2`.

AICA Core versions `v4.0` through `v4.2` keep the same protocol structure as before under the `v2` namespace. The
primary breaking change from the point of the API server and client is that the `/version` endpoint now returns the
version of AICA Core, rather than the specific version of the API server subpackage inside core. These have historically
carried the same major version, but in future the core version may have major updates without any breaking changes to
the actual API server version.

AICA Core versions `v4.3` and later introduce authentication and access scopes to the API server. An API key with
appropriate scopes is required to access the respective endpoints and functionalities.

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
