import importlib.metadata
import json
import os
import urllib.parse
from functools import wraps
from logging import INFO, getLogger
from typing import List, Optional, Union

import requests
import semver
import yaml
from deprecation import deprecated

from aica_api.sio_client import read_until

CLIENT_VERSION = importlib.metadata.version('aica_api')


class AICA:
    """API client for AICA applications."""

    # noinspection HttpUrlsUsage
    def __init__(
        self,
        url: str = 'localhost',
        port: Union[str, int] = '8080',
        log_level=INFO,
        api_key: Optional[str] = None,
    ):
        """
        Construct the API client with the address of the AICA application.

        :param url: The IP address of the AICA application
        :param port: The API port for HTTP REST endpoints (default 8080)
        :param log_level: The desired log level
        """
        if not isinstance(port, int):
            port = int(port)

        if url.startswith('http://'):
            self._address = f'{url}:{port}/api'
        elif '//' in url or ':' in url:
            raise ValueError(f'Invalid URL format {url}')
        else:
            self._address = f'http://{url}:{port}/api'

        self._logger = getLogger(__name__)
        self._logger.setLevel(log_level)
        self._protocol = None
        self._core_version = None
        self.__api_key = api_key
        self.__token = None

    def _endpoint(self, endpoint: str = '') -> str:
        """
        Build the request address for a given endpoint.

        :param endpoint: The API endpoint
        :return: The constructed request address
        """
        if self._protocol is None:
            self.protocol()
        return self.__raw_endpoint(f'{self._protocol}/{endpoint}')

    def __raw_endpoint(self, endpoint: str) -> str:
        return f'{self._address}/{endpoint}'

    def __ensure_token(self) -> None:
        """Authenticate with the API and store the result in self.__token."""
        has_version, is_compatible = self._check_version(
            None,
            '>=4.3.0',
            err_undefined=' The function call may fail due to lack of authentication.',
        )
        if not has_version or not is_compatible:
            return
        if self.__token is not None:
            return
        res = requests.post(self._endpoint('auth/login'), headers={'Authorization': f'Bearer {self.__api_key}'})
        res.raise_for_status()
        self.__token = res.json()['token']

    def _sio_auth(self) -> Optional[str]:
        # FIXME: doesn't handle token expiration
        if self.__api_key is not None:
            self.__ensure_token()
        return self.__token

    @staticmethod
    def _safe_uri(uri: str) -> str:
        """
        Make a string safe for use in a URI by encoding special characters.

        :param uri: The URI to sanitize
        :return: The sanitized URI
        """
        return urllib.parse.quote_plus(uri)

    def _request(
        self,
        method: str,
        endpoint: str,
        *,
        params: Optional[dict] = None,
        json: Optional[dict] = None,
    ) -> requests.Response:
        headers = None
        retry = 2
        res = None
        while retry > 0:
            if self.__api_key is not None:
                self.__ensure_token()
                headers = {'Authorization': f'Bearer {self.__token}'}
            res = requests.request(
                method, self._endpoint(endpoint), params=params, json=json, headers=headers, timeout=5
            )
            retry -= 1
            if res.status_code == 401:
                if self.__api_key is None:
                    break
                self.__token = None
        return res

    def _check_version(
        self,
        name: Optional[str],
        requirement: str,
        *,
        err_undefined: str = '',
        err_incompatible: Optional[str] = None,
    ) -> tuple[bool, bool]:
        fname = f'The function {name}' if name is not None else 'This function'
        if self._core_version is None and self.core_version() is None:
            self._logger.warning(
                f'{fname} requires AICA Core version {requirement}, '
                f'but the current Core version is unknown.{err_undefined}'
            )
            return False, False

        if not semver.match(self._core_version, requirement):
            if err_incompatible is not None:
                self._logger.error(
                    f'{fname} requires AICA Core version {requirement}, '
                    f'but the current AICA Core version is {self._core_version}.{err_incompatible}'
                )
            return True, False

        return True, True

    @staticmethod
    def _requires_core_version(version):
        """
        Decorator to mark a function with a specific AICA Core version constraint.
        Elides the function call and returns None with a warning if the version constraint is violated.

        Example usage:
        @_requires_core_version('>=3.2.1')
        def my_new_endpoint()
          ...

        :param version: The version constraint specifier (i.e. >=3.2.1)
        """

        def decorator(func):
            @wraps(func)
            def wrapper(self, *args, **kwargs):
                has_version, is_compatible = self._check_version(
                    func.__name__,
                    version,
                    err_undefined=' The function call behavior may be undefined.',
                    err_incompatible=' The function will not be called.',
                )
                if not is_compatible:
                    return None
                return func(self, *args, **kwargs)

            return wrapper

        return decorator

    def api_version(self) -> Union[str, None]:
        """
        Get the specific version the AICA API server as a sub-package of AICA Core

        :return: The version of the API server or None in case of connection failure
        """
        try:
            api_server_version = self.license().json()['signed_packages']['aica_api_server']
            self._logger.debug(f'AICA API server version identified as {api_server_version}')
            return api_server_version
        except requests.exceptions.RequestException:
            self._logger.error(
                f'Error connecting to the API server at {self._address}! '
                f'Check that AICA Core is running and configured with the right address.'
            )
        except KeyError as e:
            self._logger.error(
                f'Error getting version details! Expected a map of `signed_packages` to include `aica_api_server`: {e}'
            )
        return None

    def core_version(self) -> Union[str, None]:
        """
        Get the version of the AICA Core

        :return: The version of the AICA core or None in case of connection failure
        """
        core_version = None
        try:
            core_version = requests.get(self.__raw_endpoint('version')).json()
        except requests.exceptions.RequestException:
            self._logger.error(
                f'Error connecting to the API server at {self._address}! '
                f'Check that AICA Core is running and configured with the right address.'
            )

        if not semver.Version.is_valid(f'{core_version}'):
            self._logger.warning(
                f'Invalid format for the AICA Core version {core_version}! This could be a result '
                f'of an internal or pre-release build of AICA Core.'
            )
            core_version = None

        self._core_version = core_version
        return self._core_version

    @staticmethod
    def client_version() -> str:
        """
        Get the version of this API client utility

        :return: The version of the API client
        """
        return CLIENT_VERSION

    def protocol(self) -> Union[str, None]:
        """
        Get the API protocol version used as a namespace for API requests

        :return: The version of the API protocol or None in case of connection failure
        """
        try:
            self._protocol = requests.get(self.__raw_endpoint('protocol')).json()
            self._logger.debug(f'API protocol version identified as {self._protocol}')
            return self._protocol
        except requests.exceptions.RequestException:
            self._logger.error(
                f'Error connecting to the API server at {self._address}! '
                f'Check that AICA Core is running and configured with the right address.'
            )
        return None

    def check(self) -> bool:
        """
        Check if this API client is compatible with the detected AICA Core version

        :return: True if the client is compatible with the AICA Core version, False otherwise
        """
        if self._protocol is None and self.protocol() is None:
            return False
        elif self._protocol != 'v2':
            self._logger.error(
                f'The detected API protocol version {self._protocol} is not supported by this client'
                f'(v{self.client_version()}). Please refer to the compatibility table.'
            )
            return False

        if self._core_version is None and self.core_version() is None:
            return False

        version_info = semver.parse_version_info(self._core_version)

        if version_info.major == 4:
            if version_info.minor > 2 and self.__api_key is None:
                self._logger.warning(
                    f'The detected AICA Core version v{self._core_version} requires an API key for '
                    f'authentication. Please provide an API key to the client for this version.'
                )
                return False
            return True
        elif version_info.major > 4:
            self._logger.error(
                f'The detected AICA Core version v{self._core_version} is newer than the maximum AICA '
                f'Core version supported by this client (v{self.client_version()}). Please upgrade the '
                f'Python API client version for newer versions of Core.'
            )
            return False
        elif version_info.major == 3:
            self._logger.error(
                f'The detected AICA Core version v{self._core_version} is older than the minimum AICA '
                f'Core version supported by this client (v{self.client_version()}). Please downgrade '
                f'the Python API client to version v2.1.0 for API server versions v3.X.'
            )
            return False
        elif version_info.major == 2:
            self._logger.error(
                f'The detected AICA Core version v{self._core_version} is older than the minimum AICA '
                f'Core version supported by this client (v{self.client_version()}). Please downgrade '
                f'the Python API client to version v1.2.0 for API server versions v2.X.'
            )
            return False
        else:
            self._logger.error(
                f'The detected AICA Core version v{self._core_version} is deprecated and not supported '
                f'by this API client!'
            )
            return False

    def license(self) -> requests.Response:
        """
        Get licensing status details for the AICA Core session, including the type of license, a list of entitlements
        for the licensed user and a map of installed packages and versions included in the license.

        Use `license().json()` to extract the map of license details from the response object.
        """
        return self._request('GET', 'license')

    def component_descriptions(self) -> requests.Response:
        """
        Retrieve descriptions of all installed components.

        Use `component_descriptions().json()` to extract the map of descriptions from the response object.
        """
        return self._request('GET', 'components')

    def controller_descriptions(self) -> requests.Response:
        """
        Retrieve descriptions of all installed controllers.

        Use `controller_descriptions().json()` to extract the map of descriptions from the response object.
        """
        return self._request('GET', 'controllers')

    @deprecated(
        deprecated_in='3.0.0',
        removed_in='4.0.0',
        current_version=CLIENT_VERSION,
        details='Use the call_component_service function instead',
    )
    def call_service(self, component: str, service: str, payload: str) -> requests.Response:
        """
        Call a service on a component.

        :param component: The name of the component
        :param service: The name of the service
        :param payload: The service payload, formatted according to the respective service description
        """
        return self.call_component_service(component, service, payload)

    def call_component_service(self, component: str, service: str, payload: str) -> requests.Response:
        """
        Call a service on a component.

        :param component: The name of the component
        :param service: The name of the service
        :param payload: The service payload, formatted according to the respective service description
        """
        endpoint = f'application/components/{AICA._safe_uri(component)}/service/{AICA._safe_uri(service)}'
        data = {'payload': payload}
        return self._request('PUT', endpoint, json=data)

    def call_controller_service(self, hardware: str, controller: str, service: str, payload: str) -> requests.Response:
        """
        Call a service on a controller.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller
        :param service: The name of the service
        :param payload: The service payload, formatted according to the respective service description
        """
        endpoint = f'application/hardware/{AICA._safe_uri(hardware)}/controller/{AICA._safe_uri(controller)}/service/{AICA._safe_uri(service)}'
        data = {'payload': payload}
        return self._request('PUT', endpoint, json=data)

    def get_application_state(self) -> requests.Response:
        """
        Get the application state
        """
        return self._request('GET', 'application/state')

    def load_component(self, component: str) -> requests.Response:
        """
        Load a component in the current application. If the component is already loaded, or if the component is not
        described in the application, nothing happens.

        :param component: The name of the component to load
        """
        endpoint = f'application/components/{AICA._safe_uri(component)}'
        return self._request('PUT', endpoint)

    def load_controller(self, hardware: str, controller: str) -> requests.Response:
        """
        Load a controller for a given hardware interface. If the controller is already loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller to load
        """
        endpoint = f'application/hardware/{AICA._safe_uri(hardware)}/controller/{AICA._safe_uri(controller)}'
        return self._request('PUT', endpoint)

    def load_hardware(self, hardware: str) -> requests.Response:
        """
        Load a hardware interface in the current application. If the hardware interface is already loaded, or if the
        interface is not described in the application, nothing happens.

        :param hardware: The name of the hardware interface to load
        """
        endpoint = f'application/hardware/{AICA._safe_uri(hardware)}'
        return self._request('PUT', endpoint)

    def pause_application(self) -> requests.Response:
        """
        Pause the current application. This prevents any events from being triggered or handled, but
        does not pause the periodic execution of active components.
        """
        endpoint = 'application/state/transition'
        return self._request('PUT', endpoint, params={'action': 'pause'})

    def set_application(self, payload: str) -> requests.Response:
        """
        Set an application to be the current application.

        :param payload: The filepath of an application or the application content as a YAML-formatted string
        """
        if payload.endswith('.yaml') and os.path.isfile(payload):
            with open(payload, 'r') as file:
                payload = yaml.safe_load(file)
        data = {'payload': json.dumps(payload)}
        return self._request('PUT', 'application', json=data)

    def start_application(self) -> requests.Response:
        """
        Start the AICA application engine.
        """
        endpoint = 'application/state/transition?action=start'
        return self._request('PUT', endpoint)

    def stop_application(self) -> requests.Response:
        """
        Stop and reset the AICA application engine, removing all components and hardware interfaces.
        """
        endpoint = 'application/state/transition'
        return self._request('PUT', endpoint, params={'action': 'stop'})

    def set_component_parameter(
        self,
        component: str,
        parameter: str,
        value: Union[bool, int, float, bool, List[bool], List[int], List[float], List[str]],
    ) -> requests.Response:
        """
        Set a parameter on a component.

        :param component: The name of the component
        :param parameter: The name of the parameter
        :param value: The value of the parameter
        """
        endpoint = f'application/components/{AICA._safe_uri(component)}/parameter/{AICA._safe_uri(parameter)}'
        data = {'value': value}
        return self._request('PUT', endpoint, json=data)

    def set_controller_parameter(
        self,
        hardware: str,
        controller: str,
        parameter: str,
        value: Union[bool, int, float, bool, List[bool], List[int], List[float], List[str]],
    ) -> requests.Response:
        """
        Set a parameter on a controller.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller
        :param parameter: The name of the parameter
        :param value: The value of the parameter
        """
        endpoint = f'application/hardware/{AICA._safe_uri(hardware)}/controller/{AICA._safe_uri(controller)}/parameter/{AICA._safe_uri(parameter)}'
        data = {'value': value}
        return self._request('PUT', endpoint, json=data)

    def set_lifecycle_transition(self, component: str, transition: str) -> requests.Response:
        """
        Trigger a lifecycle transition on a component. The transition label must be one of the following:
        ['configure', 'activate', 'deactivate', 'cleanup', 'unconfigured_shutdown', 'inactive_shutdown',
        'acitve_shutdown']

        The transition will only be executed if the target is a lifecycle component and the current lifecycle state
        allows the requested transition.

        :param component: The name of the component
        :param transition: The lifecycle transition label
        """
        endpoint = f'application/components/{AICA._safe_uri(component)}/lifecycle/transition'
        data = {'transition': transition}
        return self._request('PUT', endpoint, json=data)

    def switch_controllers(
        self,
        hardware: str,
        activate: Union[None, List[str]] = None,
        deactivate: Union[None, List[str]] = None,
    ) -> requests.Response:
        """
        Activate and deactivate the controllers for a given hardware interface.

        :param hardware: The name of the hardware interface
        :param activate: A list of controllers to activate
        :param deactivate: A list of controllers to deactivate
        """
        endpoint = f'application/hardware/{AICA._safe_uri(hardware)}/controllers'
        params = {
            'activate': [] if not activate else activate,
            'deactivate': [] if not deactivate else deactivate,
        }
        return self._request('PUT', endpoint, params=params)

    def unload_component(self, component: str) -> requests.Response:
        """
        Unload a component in the current application. If the component is not loaded, or if the component is not
        described in the application, nothing happens.

        :param component: The name of the component to unload
        """
        endpoint = f'application/components/{AICA._safe_uri(component)}'
        return self._request('DELETE', endpoint)

    def unload_controller(self, hardware: str, controller: str) -> requests.Response:
        """
        Unload a controller for a given hardware interface. If the controller is not loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller to unload
        """
        endpoint = f'application/hardware/{AICA._safe_uri(hardware)}/controller/{AICA._safe_uri(controller)}'
        return self._request('DELETE', endpoint)

    def unload_hardware(self, hardware: str) -> requests.Response:
        """
        Unload a hardware interface in the current application. If the hardware interface is not loaded, or if the
        interface is not described in the application, nothing happens.

        :param hardware: The name of the hardware interface to unload
        """
        endpoint = f'application/hardware/{AICA._safe_uri(hardware)}'
        return self._request('DELETE', endpoint)

    def get_application(self) -> requests.Response:
        """
        Get the currently set application
        """
        return self._request('GET', 'application')

    @_requires_core_version('>=4.0.0')
    def manage_sequence(self, sequence_name: str, action: str):
        """
        Manage a sequence. The action label must be one of the following: ['start', 'restart', 'abort']

        The action will only be executed if the sequence exists and allows the requested action.

        :param sequence_name: The name of the sequence
        :param action: The sequence action label
        """
        endpoint = f'application/sequences/{AICA._safe_uri(sequence_name)}'
        return self._request('PUT', endpoint, params={'action': AICA._safe_uri(action)})

    def wait_for_component(self, component: str, state: str, timeout: Union[None, int, float] = None) -> bool:
        """
        Wait for a component to be in a particular state. Components can be in any of the following states:
            ['unloaded', 'loaded', 'unconfigured', 'inactive', 'active', 'finalized']

        :param component: The name of the component
        :param state: The state of the component to wait for
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the component is in the intended state before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[component]['state'] == state,
                url=self._address,
                namespace='/v2/components',
                event='component_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    @_requires_core_version('>=3.1.0')
    def wait_for_hardware(self, hardware: str, state: str, timeout: Union[None, int, float] = None) -> bool:
        """
        Wait for a hardware interface to be in a particular state. Hardware can be in any of the following states:
            ['unloaded', 'loaded']

        :param hardware: The name of the hardware interface
        :param state: The state of the hardware to wait for
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the hardware is in the intended state before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[hardware]['state'] == state,
                url=self._address,
                namespace='/v2/hardware',
                event='hardware_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    @_requires_core_version('>=3.1.0')
    def wait_for_controller(
        self,
        hardware: str,
        controller: str,
        state: str,
        timeout: Union[None, int, float] = None,
    ) -> bool:
        """
        Wait for a controller to be in a particular state. Controllers can be in any of the following states:
            ['unloaded', 'loaded', 'active', 'finalized']

        :param hardware: The name of the hardware interface responsible for the controller
        :param controller: The name of the controller
        :param state: The state of the controller to wait for
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the controller is in the intended state before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[hardware]['controllers'][controller]['state'] == state,
                url=self._address,
                namespace='/v2/hardware',
                event='hardware_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    @_requires_core_version('>=3.1.0')
    def wait_for_component_predicate(
        self, component: str, predicate: str, timeout: Union[None, int, float] = None
    ) -> bool:
        """
        Wait until a component predicate is true.

        :param component: The name of the component
        :param predicate: The name of the predicate
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the predicate is true before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[component]['predicates'][predicate],
                url=self._address,
                namespace='/v2/components',
                event='component_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    @_requires_core_version('>=3.1.0')
    def wait_for_controller_predicate(
        self,
        hardware: str,
        controller: str,
        predicate: str,
        timeout: Union[None, int, float] = None,
    ) -> bool:
        """
        Wait until a controller predicate is true.

        :param hardware: The name of the hardware interface responsible for the controller
        :param controller: The name of the controller
        :param predicate: The name of the predicate
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the predicate is true before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[hardware]['controllers'][controller]['predicates'][predicate],
                url=self._address,
                namespace='/v2/hardware',
                event='hardware_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    def wait_for_condition(self, condition: str, timeout=None) -> bool:
        """
        Wait until a condition is true.

        :param condition: The name of the condition
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the condition is true before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[condition],
                url=self._address,
                namespace='/v2/conditions',
                event='conditions',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    @_requires_core_version('>=4.0.0')
    def wait_for_sequence(self, sequence: str, state: str, timeout=None) -> bool:
        """
        Wait for a sequence to be in a particular state. Sequences can be in any of the following states:
            ['active', 'inactive', 'aborted']

        :param sequence: The name of the sequence
        :param state: The state of the sequence to wait for
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the condition is true before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[sequence]['state'] == state,
                url=self._address,
                namespace='/v2/sequences',
                event='sequences',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )
