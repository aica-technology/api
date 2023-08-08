from typing import Union, List

import requests

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
        return f'{self._address}/v1/{endpoint}'

    def _ws_endpoint(self, endpoint=''):
        """
        Build the connection address for a given websocket endpoint.

        :param endpoint: The websocket endpoint
        :return: The constructed connection address
        """
        return f'{self._ws_address}/{endpoint}'

    def check(self) -> requests.Response:
        """
        Make a GET request to the default endpoint to verify connectivity.
        """
        return requests.get(self._endpoint())

    def component_descriptions(self) -> requests.Response:
        """
        Retrieve the JSON descriptions of all available components.
        """
        return requests.get(self._endpoint('components'))

    def call_service(self, component_name: str, service_name: str,
                     payload: str) -> requests.Response:
        """
        Call a service on a component.

        :param component_name: The name of the component
        :param service_name: The name of the service
        :param payload: The service payload, formatted according to the respective service description
        """
        endpoint = 'application/components/' + component_name + '/service/' + service_name
        data = {"payload": payload}
        return requests.put(self._endpoint(endpoint), json=data)

    # TODO: Missing v1 endpoint
    def init_application(self, auto_load_hardware: bool = True) -> requests.Response:
        """
        Initialize the currently set application.

        :param auto_load_hardware: If true, load hardware interfaces automatically when initializing the application.
        """
        return requests.post(self._endpoint('init_application'), json={"auto_load_hardware": auto_load_hardware})

    def get_application_state(self) -> requests.Response:
        """
        Get the application state
        """
        return requests.get(self._endpoint('application/state'))

    def load_component(self, component_name: str) -> requests.Response:
        """
        Load a component in the current application. If the component is already loaded, or if the component is not
        described in the application, nothing happens.

        :param component_name: The name of the component to load
        """
        endpoint = 'application/components/' + component_name
        return requests.put(self._endpoint(endpoint))

    def load_controller(self, interface_name: str, controller_name: str) -> requests.Response:
        """
        Load a controller for a given hardware interface. If the controller is already loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        :param interface_name: The name of the hardware interface
        :param controller_name: The name of the controller to load
        """
        endpoint = 'application/hardware/' + interface_name + '/controller/' + controller_name
        return requests.put(self._endpoint(endpoint))

    def load_hardware(self, interface_name: str) -> requests.Response:
        """
        Load a hardware interface in the current application. If the hardware interface is already loaded, or if the
        interface is not described in the application, nothing happens.

        :param interface_name: The name of the hardware interface to load
        """
        endpoint = 'application/hardware/' + interface_name
        return requests.put(self._endpoint(endpoint))

    def pause_application(self) -> requests.Response:
        """
        Pause the current application. This prevents any events from being triggered or handled, but
        does not pause the periodic execution of active components.
        """
        endpoint = 'application/state?action=pause'
        return requests.put(self._endpoint(endpoint))

    def set_application(self, payload: str) -> requests.Response:
        """
        Set an application to be the current application.

        :param payload: The filepath of an application on the AICA computer, or the application content as a
        YAML-formatted string
        """
        data = {
            "payload": payload
        }
        return requests.put(self._endpoint('application'), json=data)

    def start_application(self) -> requests.Response:
        """
        Start the AICA application engine.
        """
        endpoint = 'application/state?action=start'
        return requests.put(self._endpoint(endpoint))

    def stop_application(self) -> requests.Response:
        """
        Stop and reset the AICA application engine, removing all components and hardware interfaces.
        """
        endpoint = 'application/state?action=stop'
        return requests.put(self._endpoint(endpoint))

    def set_parameter(self, component_name: str, parameter_name: str, parameter_value: str) -> requests.Response:
        """
        Set a parameter on a component.

        :param component_name: The name of the component
        :param parameter_name: The name of the parameter
        :param parameter_value: The value of the parameter
        """
        endpoint = 'application/components/' + component_name + '/parameter/' + parameter_name
        data = {"parameter_value": parameter_value}
        return requests.put(self._endpoint(endpoint), json=data)

    def switch_controllers(self, interface_name: str, start: Union[None, List[str]] = None,
                           stop: Union[None, List[str]] = None) -> requests.Response:
        """
        Start and stop the controllers for a given hardware interface.

        :param interface_name: The name of the hardware interface to unload
        :param start: A list of controllers to start
        :param stop: A list of controllers to stop
        """
        endpoint = 'application/controllers'
        params = {
            "interface_name": interface_name,
            "start": [] if not start else start,
            "stop": [] if not stop else stop
        }
        return requests.put(self._endpoint(endpoint), params=params)

    def unload_component(self, component_name: str) -> requests.Response:
        """
        Unload a component in the current application. If the component is not loaded, or if the component is not
        described in the application, nothing happens.

        :param component_name: The name of the component to unload
        """
        endpoint = 'application/components/' + component_name
        return requests.delete(self._endpoint(endpoint))

    def unload_controller(self, interface_name: str, controller_name: str) -> requests.Response:
        """
        Unload a controller for a given hardware interface. If the controller is not loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        :param interface_name: The name of the hardware interface
        :param controller_name: The name of the controller to unload
        """
        endpoint = 'application/hardware/' + interface_name + '/controller/' + controller_name
        return requests.delete(self._endpoint(endpoint))

    def unload_hardware(self, interface_name: str) -> requests.Response:
        """
        Unload a hardware interface in the current application. If the hardware interface is not loaded, or if the
        interface is not described in the application, nothing happens.

        :param interface_name: The name of the hardware interface to unload
        """
        endpoint = 'application/hardware/' + interface_name
        return requests.delete(self._endpoint(endpoint))

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
