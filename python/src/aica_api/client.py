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
        return f'{self._address}/{endpoint}'

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
        return requests.get(self._endpoint('component_descriptions'))

    def call_service(self, component_name: str, service_name: str,
                     payload: Union[None, str] = None) -> requests.Response:
        """
        Call a service on a component.

        :param component_name: The name of the component
        :param service_name: The name of the service
        :param payload: The optional service payload, formatted according to the respective service description
        """
        request = {"component_name": component_name, "service_name": service_name}
        if payload:
            request["payload"] = payload
        return requests.post(self._endpoint('call_service'), json=request)

    def init_application(self, auto_load_hardware: bool = True) -> requests.Response:
        """
        Initialize the currently set application.

        :param auto_load_hardware: If true, load hardware interfaces automatically when initializing the application.
        """
        return requests.post(self._endpoint('init_application'), json={"auto_load_hardware": auto_load_hardware})

    def load_component(self, component_name: str) -> requests.Response:
        """
        Load a component in the current application. If the component is already loaded, or if the component is not
        described in the application, nothing happens.

        :param component_name: The name of the component to load
        """
        return requests.post(self._endpoint('load_component'), json={"component_name": component_name})

    def load_controller(self, interface_name: str, controller_name: str) -> requests.Response:
        """
        Load a controller for a given hardware interface. If the controller is already loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        :param interface_name: The name of the hardware interface
        :param controller_name: The name of the controller to load
        """
        return requests.post(self._endpoint('load_controller'),
                             json={"interface_name": interface_name, "controller_name": controller_name})

    def load_hardware(self, interface_name: str) -> requests.Response:
        """
        Load a hardware interface in the current application. If the hardware interface is already loaded, or if the
        interface is not described in the application, nothing happens.

        :param interface_name: The name of the hardware interface to load
        """
        return requests.post(self._endpoint('load_hardware'), json={"interface_name": interface_name})

    def pause_application(self) -> requests.Response:
        """
        Pause the current application. This prevents any events from being triggered or handled, but
        does not pause the periodic execution of active components.
        """
        return requests.post(self._endpoint('pause_application'))

    def set_application(self, payload: str) -> requests.Response:
        """
        Set an application to be the current application.

        :param payload: The filepath of an application on the AICA computer, or the application content as a
        YAML-formatted string
        """
        return requests.post(self._endpoint('set_application'), json={"payload": payload})

    def start_application(self) -> requests.Response:
        """
        Start the AICA application engine.
        """
        return requests.post(self._endpoint('start_application'))

    def stop_application(self) -> requests.Response:
        """
        Stop and reset the AICA application engine, removing all components and hardware interfaces.
        """
        return requests.post(self._endpoint('stop_application'))

    def switch_controllers(self, interface_name: str, start: Union[None, List[str]] = None,
                           stop: Union[None, List[str]] = None) -> requests.Response:
        """
        Start and stop the controllers for a given hardware interface.

        :param interface_name: The name of the hardware interface to unload
        :param start: A list of controllers to start
        :param stop: A list of controllers to stop
        """
        return requests.post(self._endpoint('call_service'),
                             json={"interface_name": interface_name, "start": [] if not start else start,
                                   "stop": [] if not stop else stop})

    def unload_component(self, component_name: str) -> requests.Response:
        """
        Unload a component in the current application. If the component is not loaded, or if the component is not
        described in the application, nothing happens.

        :param component_name: The name of the component to unload
        """
        return requests.post(self._endpoint('unload_component'), json={"component_name": component_name})

    def unload_controller(self, interface_name: str, controller_name: str) -> requests.Response:
        """
        Unload a controller for a given hardware interface. If the controller is not loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        :param interface_name: The name of the hardware interface
        :param controller_name: The name of the controller to unload
        """
        return requests.post(self._endpoint('load_controller'),
                             json={"interface_name": interface_name, "controller_name": controller_name})

    def unload_hardware(self, interface_name: str) -> requests.Response:
        """
        Unload a hardware interface in the current application. If the hardware interface is not loaded, or if the
        interface is not described in the application, nothing happens.

        :param interface_name: The name of the hardware interface to unload
        """
        return requests.post(self._endpoint('unload_hardware'), json={"interface_name": interface_name})

    def wait_for_predicate(self, component, predicate, timeout: Union[None, int, float] = None):
        """
        Wait until a component predicate is true.

        :param component: The name of the component
        :param predicate: The name of the predicate
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: False if the connection times out before the predicate is true
        """
        component = f'/{component}'

        def check_predicate(data):
            try:
                if data[component][predicate]:
                    return True
            except KeyError:
                return False

        ws = WebsocketSyncClient(self._ws_endpoint('predicates'))
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
