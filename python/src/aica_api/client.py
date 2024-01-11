from typing import Union, List

import os
import requests
import yaml

from aica_api.ws_client import WebsocketSyncClient


class AICA:
    """
    API client for AICA applications.
    """

    # noinspection HttpUrlsUsage
    def __init__(self, url: str = 'localhost', port: Union[str, int] = '5000'):
        """
        Construct the API client with the address of the AICA application.

        :param url: The IP address of the AICA application
        :param port: The API port for HTTP REST endpoints (default 5000)
        """
        if not isinstance(port, int):
            port = int(port)

        if url.startswith('http://'):
            self._address = f'{url}:{port}'
        elif '//' in url or ':' in url:
            raise ValueError(f'Invalid URL format {url}')
        else:
            self._address = f'http://{url}:{port}'
            self._ws_address = f'ws://{url}:{port}'

    def _endpoint(self, endpoint=''):
        """
        Build the request address for a given endpoint.

        :param endpoint: The API endpoint
        :return: The constructed request address
        """
        return f'{self._address}/v2/{endpoint}'

    def _ws_endpoint(self, endpoint=''):
        """
        Build the connection address for a given websocket endpoint.

        :param endpoint: The websocket endpoint
        :return: The constructed connection address
        """
        return f'{self._ws_address}/{endpoint}'

    def check(self) -> bool:
        """
        Check if the API version is v2 (any v2.x.x tag)
        """
        # TODO: come up with a compatibility table in the future
        try:
            api_version = requests.get(f'{self._address}/version').json()
        except requests.exceptions.RequestException as e:
            print(f'Error connecting to the API! {e}')
            return False
        new_version = api_version.startswith('2')
        if not new_version:
            print(f'The detected API version v{api_version} is older than the minimum API version v2.0.0 supported by '
                  f'this client')
        return new_version

    def component_descriptions(self) -> requests.Response:
        """
        Retrieve the JSON descriptions of all available components.
        """
        return requests.get(self._endpoint('components'))

    def controller_descriptions(self) -> requests.Response:
        """
        Retrieve the JSON descriptions of all available controllers.
        """
        return requests.get(self._endpoint('controllers'))

    def call_service(self, component: str, service: str, payload: str) -> requests.Response:
        """
        Call a service on a component.

        :param component: The name of the component
        :param service: The name of the service
        :param payload: The service payload, formatted according to the respective service description
        """
        endpoint = 'application/components/' + component + '/service/' + service
        data = {"payload": payload}
        return requests.put(self._endpoint(endpoint), json=data)

    def get_application_state(self) -> requests.Response:
        """
        Get the application state
        """
        return requests.get(self._endpoint('application/state'))

    def load_component(self, component: str) -> requests.Response:
        """
        Load a component in the current application. If the component is already loaded, or if the component is not
        described in the application, nothing happens.

        :param component: The name of the component to load
        """
        endpoint = 'application/components/' + component
        return requests.put(self._endpoint(endpoint))

    def load_controller(self, hardware: str, controller: str) -> requests.Response:
        """
        Load a controller for a given hardware interface. If the controller is already loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller to load
        """
        endpoint = 'application/hardware/' + hardware + '/controller/' + controller
        return requests.put(self._endpoint(endpoint))

    def load_hardware(self, hardware: str) -> requests.Response:
        """
        Load a hardware interface in the current application. If the hardware interface is already loaded, or if the
        interface is not described in the application, nothing happens.

        :param hardware: The name of the hardware interface to load
        """
        endpoint = 'application/hardware/' + hardware
        return requests.put(self._endpoint(endpoint))

    def pause_application(self) -> requests.Response:
        """
        Pause the current application. This prevents any events from being triggered or handled, but
        does not pause the periodic execution of active components.
        """
        endpoint = 'application/state/transition?action=pause'
        return requests.put(self._endpoint(endpoint))

    def set_application(self, payload: str) -> requests.Response:
        """
        Set an application to be the current application.

        :param payload: The filepath of an application or the application content as a YAML-formatted string
        """
        if payload.endswith(".yaml") and os.path.isfile(payload):
            with open(payload, "r") as file:
                payload = yaml.safe_load(file)
        data = {
            "payload": payload
        }
        return requests.put(self._endpoint('application'), json=data)

    def start_application(self) -> requests.Response:
        """
        Start the AICA application engine.
        """
        endpoint = 'application/state/transition?action=start'
        return requests.put(self._endpoint(endpoint))

    def stop_application(self) -> requests.Response:
        """
        Stop and reset the AICA application engine, removing all components and hardware interfaces.
        """
        endpoint = 'application/state/transition?action=stop'
        return requests.put(self._endpoint(endpoint))

    def set_component_parameter(self, component: str, parameter: str, value: Union[
            bool, int, float, bool, List[bool], List[int], List[float], List[str]]) -> requests.Response:
        """
        Set a parameter on a component.

        :param component: The name of the component
        :param parameter: The name of the parameter
        :param value: The value of the parameter
        """
        endpoint = 'application/components/' + component + '/parameter/' + parameter
        data = {"value": value}
        return requests.put(self._endpoint(endpoint), json=data)

    def set_controller_parameter(self, hardware: str, controller: str, parameter: str, value: Union[
            bool, int, float, bool, List[bool], List[int], List[float], List[str]]) -> requests.Response:
        """
        Set a parameter on a controller.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller
        :param parameter: The name of the parameter
        :param value: The value of the parameter
        """
        endpoint = 'application/hardware/' + hardware + '/controller/' + controller + '/parameter/' + parameter
        data = {"value": value}
        return requests.put(self._endpoint(endpoint), json=data)

    def set_lifecycle_transition(self, component: str, transition: str) -> requests.Response:
        """
        Trigger a lifecycle transition on a component. The transition label must be one of the following:
        ['configure', 'activate', 'deactivate', 'cleanup', 'shutdown']

        The transition will only be executed if the target is a lifecycle component and the current lifecycle state
        allows the requested transition.

        :param component: The name of the component
        :param transition: The lifecycle transition label
        """
        endpoint = 'application/components/' + component + '/lifecycle/transition'
        data = {"transition": transition}
        return requests.put(self._endpoint(endpoint), json=data)

    def switch_controllers(self, hardware: str, activate: Union[None, List[str]] = None,
                           deactivate: Union[None, List[str]] = None) -> requests.Response:
        """
        Activate and deactivate the controllers for a given hardware interface.

        :param hardware: The name of the hardware interface
        :param activate: A list of controllers to activate
        :param deactivate: A list of controllers to deactivate
        """
        endpoint = 'application/hardware/' + hardware + '/controllers'
        params = {
            "activate": [] if not activate else activate,
            "deactivate": [] if not deactivate else deactivate
        }
        return requests.put(self._endpoint(endpoint), params=params)

    def unload_component(self, component: str) -> requests.Response:
        """
        Unload a component in the current application. If the component is not loaded, or if the component is not
        described in the application, nothing happens.

        :param component: The name of the component to unload
        """
        endpoint = 'application/components/' + component
        return requests.delete(self._endpoint(endpoint))

    def unload_controller(self, hardware: str, controller: str) -> requests.Response:
        """
        Unload a controller for a given hardware interface. If the controller is not loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller to unload
        """
        endpoint = 'application/hardware/' + hardware + '/controller/' + controller
        return requests.delete(self._endpoint(endpoint))

    def unload_hardware(self, hardware: str) -> requests.Response:
        """
        Unload a hardware interface in the current application. If the hardware interface is not loaded, or if the
        interface is not described in the application, nothing happens.

        :param hardware: The name of the hardware interface to unload
        """
        endpoint = 'application/hardware/' + hardware
        return requests.delete(self._endpoint(endpoint))

    def get_application(self):
        """
        Get the application
        """
        endpoint = "application"
        return requests.get(self._endpoint(endpoint))

    def wait_for_predicate(self, component, predicate, timeout: Union[None, int, float] = None):
        """
        Wait until a component predicate is true.

        :param component: The name of the component
        :param predicate: The name of the predicate
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: False if the connection times out before the predicate is true
        """
        component = f'{component}'

        def check_predicate(data):
            try:
                if data[component]["predicates"][predicate]:
                    return True
            except KeyError:
                return False

        ws = WebsocketSyncClient(self._ws_endpoint('components'))
        return ws.read_until(check_predicate, timeout=timeout)

    def wait_for_condition(self, condition, timeout=None):
        """
        Wait until a condition is true.

        :param condition: The name of the condition
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: False if the connection times out before the condition is true
        """

        def check_condition(data):
            try:
                if data[condition]:
                    return True
            except KeyError:
                return False

        ws = WebsocketSyncClient(self._ws_endpoint('conditions'))
        return ws.read_until(check_condition, timeout=timeout)
