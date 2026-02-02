import importlib.metadata
import os
from functools import wraps
from logging import INFO, getLogger
from typing import Callable, List, Optional, TypeAlias, TypeVar, Union

import httpx
import semver
from deprecation import deprecated

from aica_api.errors import APIError
from aica_api.sdk.sdk import Client, errors
from aica_api.sdk.sdk.api.api import get_api_version
from aica_api.sdk.sdk.api.applications import get_applications
from aica_api.sdk.sdk.api.auth import login
from aica_api.sdk.sdk.api.descriptions import (
    get_component_descriptions,
    get_controller_descriptions,
    get_extension_descriptions,
)
from aica_api.sdk.sdk.api.engine import (
    call_component_service,
    call_controller_service,
    get_application_state,
    get_current_application,
    load_component,
    load_controller,
    load_hardware,
    set_component_parameter,
    set_controller_parameter,
    set_current_application,
    set_current_application_by_id,
    switch_controllers,
    trigger_application_transition,
    trigger_component_transition,
    trigger_sequence_transition,
    unload_component,
    unload_controller,
    unload_hardware,
)
from aica_api.sdk.sdk.api.licensing import get_core_version
from aica_api.sdk.sdk.client import AuthenticatedClient
from aica_api.sdk.sdk.models.application_lifecycle_transition import ApplicationLifecycleTransition
from aica_api.sdk.sdk.models.application_lifecycle_transition_transition import ApplicationLifecycleTransitionTransition
from aica_api.sdk.sdk.models.application_status import ApplicationStatus
from aica_api.sdk.sdk.models.call_component_service import CallComponentService
from aica_api.sdk.sdk.models.call_controller_service import CallControllerService
from aica_api.sdk.sdk.models.component_descriptions import ComponentDescriptions
from aica_api.sdk.sdk.models.component_lifecycle_transition import ComponentLifecycleTransition
from aica_api.sdk.sdk.models.component_lifecycle_transition_transition import (
    ComponentLifecycleTransitionTransition as LifecycleTransition,
)
from aica_api.sdk.sdk.models.component_reference import ComponentReference
from aica_api.sdk.sdk.models.controller_descriptions import ControllerDescriptions
from aica_api.sdk.sdk.models.current_application import CurrentApplication
from aica_api.sdk.sdk.models.error_response import ErrorResponse
from aica_api.sdk.sdk.models.extension_descriptions import ExtensionDescriptions
from aica_api.sdk.sdk.models.hardware_reference import HardwareReference
from aica_api.sdk.sdk.models.load_controller_request import LoadControllerRequest
from aica_api.sdk.sdk.models.sequence_lifecycle_transition import SequenceLifecycleTransition
from aica_api.sdk.sdk.models.sequence_lifecycle_transition_transition import (
    SequenceLifecycleTransitionTransition as SequenceTransition,
)
from aica_api.sdk.sdk.models.set_component_parameter import SetComponentParameter
from aica_api.sdk.sdk.models.set_controller_parameter import SetControllerParameter
from aica_api.sdk.sdk.models.set_current_application import SetCurrentApplication
from aica_api.sdk.sdk.models.switch_controllers_request import SwitchControllersRequest
from aica_api.sdk.sdk.types import UNSET
from aica_api.sio_client import read_until

CLIENT_VERSION = importlib.metadata.version('aica_api')

T = TypeVar('T')
ValueParameterT: TypeAlias = Union[bool, int, float, str, list[bool], list[int], list[float], list[str]]


class AICA:
    """API client for AICA applications."""

    def __init__(
        self,
        *,
        api_key: str,
        url: str = 'http://localhost:8080/api',
        log_level=INFO,
    ):
        """
        Construct the API client with the address of the AICA application.

        :param url: The URL of the AICA Core instance
        :param api_key: The API key for authentication
        :param log_level: The desired log level
        """
        self.__address = url

        self._logger = getLogger(__name__)
        self._logger.setLevel(log_level)
        self._protocol = None
        self._core_version = None

        self.__raw_client = Client(base_url=self.__address, raise_on_unexpected_status=True)
        self.__client: AuthenticatedClient | None = None

        self.__api_key = api_key
        self.__token = None

    def __handle_errors(self, do: Callable[[], T | ErrorResponse | None], *, expect_empty: bool = False) -> T:
        try:
            res = do()
            if isinstance(res, ErrorResponse):
                raise APIError(f'API error {res.error.code}: {res.error.message}')
            if res is None:
                if expect_empty:
                    return None  # type: ignore
                raise APIError('Expected a valid response, but got None')
            return res
        except errors.UnexpectedStatus as e:
            raise APIError('Unexpected status') from e
        except httpx.TimeoutException as e:
            raise APIError('Timeout') from e

    def __log_api_error(self, e: APIError):
        self._logger.error(
            f'Error connecting to the API server at {self.__address}! '
            f'Check that AICA Core is running and configured with the right address. '
            f'Error: {e}'
        )

    def __ensure_token(self) -> str:
        """Authenticate with the API and store the result in self.__token."""
        if self.__token is not None:
            return self.__token

        protocol = self.protocol()
        if protocol != 'v3':
            raise APIError(f'Mismatched protocol version (expected v3, got {protocol})')

        res = self.__handle_errors(
            lambda: login.sync(
                client=AuthenticatedClient(
                    base_url=self.__address, raise_on_unexpected_status=True, token=self.__api_key
                )
            )
        )
        self.__token = res.token
        return res.token

    def __get_client(self) -> AuthenticatedClient:
        if self.__client is not None:
            return self.__client
        token = self.__ensure_token()
        client = AuthenticatedClient(
            base_url=self.__address,
            raise_on_unexpected_status=True,
            token=token,
        )
        self.__client = client
        return client

    def _sio_auth(self) -> Optional[str]:
        # FIXME: doesn't handle token expiration
        if self.__api_key is not None:
            self.__ensure_token()
        return self.__token

    def __check_version(
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
                _, is_compatible = self.__check_version(
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

    def core_version(self) -> Union[str, None]:
        """
        Get the version of the AICA Core

        Raises:
            aica_api.client.APIError: If the API call fails.

        :return: The version of the AICA core or None in case of connection failure
        """
        core_version = None
        try:
            core_version = self.__handle_errors(lambda: get_core_version.sync(client=self.__get_client())).core
        except APIError as e:
            self.__log_api_error(e)

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

        Raises:
            aica_api.client.APIError: If the API call fails.

        :return: The version of the API protocol or None in case of connection failure
        """
        try:
            self._protocol = self.__handle_errors(lambda: get_api_version.sync(client=self.__raw_client))
            self._logger.debug(f'API protocol version identified as {self._protocol}')
            return self._protocol
        except APIError as e:
            self.__log_api_error(e)
        return None

    def check(self) -> bool:
        """
        Check if this API client is compatible with the detected AICA Core version

        :return: True if the client is compatible with the AICA Core version, False otherwise
        """
        if self._protocol is None and self.protocol() is None:
            return False
        elif self._protocol != 'v3':
            self._logger.error(
                f'The detected API protocol version {self._protocol} is not supported by this client'
                f' (v{self.client_version()}). Please refer to the compatibility table.'
            )
            return False

        if self._core_version is None and self.core_version() is None:
            return False

        version_info = semver.parse_version_info(self._core_version)

        if version_info.major == 5:
            if version_info.minor >= 1:
                return True
            else:
                self._logger.error(
                    f'The detected AICA Core version v{self._core_version} is older than the minimum AICA '
                    f'Core version supported by this client (v{self.client_version()}). Please upgrade the '
                    f'AICA Core instance to a newer version.'
                )
                return False
        elif version_info.major > 5:
            self._logger.error(
                f'The detected AICA Core version v{self._core_version} is newer than the maximum AICA '
                f'Core version supported by this client (v{self.client_version()}). Please upgrade the '
                f'Python API client version for newer versions of Core.'
            )
            return False
        else:
            self._logger.error(
                f'The detected AICA Core version v{self._core_version} is deprecated and not supported '
                f'by this API client!'
            )
            return False

    def extension_descriptions(self) -> ExtensionDescriptions:
        """
        Retrieve descriptions of all installed extensions.

        Raises:
            aica_api.client.APIError: If the API call fails.
        """
        return self.__handle_errors(lambda: get_extension_descriptions.sync(client=self.__get_client()))

    @deprecated(
        deprecated_in='4.0.0',
        removed_in='5.0.0',
        current_version=CLIENT_VERSION,
        details='Use the extension_descriptions function instead',
    )
    def component_descriptions(self) -> ComponentDescriptions:
        """
        Retrieve descriptions of all installed components.

        Raises:
            aica_api.client.APIError: If the API call fails.
        """
        return self.__handle_errors(lambda: get_component_descriptions.sync(client=self.__get_client()))

    @deprecated(
        deprecated_in='4.0.0',
        removed_in='5.0.0',
        current_version=CLIENT_VERSION,
        details='Use the extension_descriptions function instead',
    )
    def controller_descriptions(self) -> ControllerDescriptions:
        """
        Retrieve descriptions of all installed controllers.

        Raises:
            aica_api.client.APIError: If the API call fails.
        """
        return self.__handle_errors(lambda: get_controller_descriptions.sync(client=self.__get_client()))

    def call_component_service(self, component: str, service: str, payload: Optional[str] = None) -> None:
        """
        Call a service on a component.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param component: The name of the component
        :param service: The name of the service
        :param payload: The service payload, formatted according to the respective service description
        """
        self.__handle_errors(
            lambda: call_component_service.sync(
                client=self.__get_client(),
                body=CallComponentService(component=component, service=service, payload=payload if payload else UNSET),
            ),
            expect_empty=True,
        )

    def call_controller_service(
        self, hardware: str, controller: str, service: str, payload: Optional[str] = None
    ) -> None:
        """
        Call a service on a controller.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller
        :param service: The name of the service
        :param payload: The service payload, formatted according to the respective service description
        """
        self.__handle_errors(
            lambda: call_controller_service.sync(
                client=self.__get_client(),
                body=CallControllerService(
                    hardware=hardware, controller=controller, service=service, payload=payload if payload else UNSET
                ),
            ),
            expect_empty=True,
        )

    def get_application_state(self) -> ApplicationStatus:
        """
        Get the application state

        Raises:
            aica_api.client.APIError: If the API call fails.
        """
        return self.__handle_errors(lambda: get_application_state.sync(client=self.__get_client())).status

    def load_component(self, component: str) -> None:
        """
        Load a component in the current application. If the component is already loaded, or if the component is not
        described in the application, nothing happens.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param component: The name of the component to load
        """
        self.__handle_errors(
            lambda: load_component.sync(client=self.__get_client(), body=ComponentReference(component=component)),
            expect_empty=True,
        )

    def load_controller(self, hardware: str, controller: str) -> None:
        """
        Load a controller for a given hardware interface. If the controller is already loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller to load
        """
        self.__handle_errors(
            lambda: load_controller.sync(
                client=self.__get_client(), body=LoadControllerRequest(hardware=hardware, controller=controller)
            ),
            expect_empty=True,
        )

    def load_hardware(self, hardware: str) -> None:
        """
        Load a hardware interface in the current application. If the hardware interface is already loaded, or if the
        interface is not described in the application, nothing happens.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface to load
        """
        self.__handle_errors(
            lambda: load_hardware.sync(client=self.__get_client(), body=HardwareReference(hardware=hardware)),
            expect_empty=True,
        )

    def __start_application_transition(self, transition: ApplicationLifecycleTransitionTransition) -> None:
        self.__handle_errors(
            lambda: trigger_application_transition.sync(
                client=self.__get_client(), body=ApplicationLifecycleTransition(transition=transition)
            ),
            expect_empty=True,
        )

    def pause_application_events(self) -> None:
        """
        Pause the event handler for a running application.
        This prevents any new events from being triggered or handled, but does not pause the periodic execution of active components or controllers.
        Paused events are placed in a queue and actioned when the event handler is resumed.

        The event handler can be resumed using the start_application method.

        Raises:
            aica_api.client.APIError: If the API call fails.
        """
        self.__start_application_transition(ApplicationLifecycleTransitionTransition.PAUSE)

    def set_application(self, payload: str) -> None:
        """
        Set an application to be the current application.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param payload: The filepath of an application or the application content as a YAML-formatted string
        """
        if (payload.endswith('.yaml') or payload.endswith('.yml')) and os.path.isfile(payload):
            with open(payload, 'r') as file:
                payload = file.read()
        self.__handle_errors(
            lambda: set_current_application.sync(client=self.__get_client(), body=SetCurrentApplication(yaml=payload)),
            expect_empty=True,
        )

    def load_application(self, name: str) -> None:
        """
        Load an application by name from the applications directory.

        Raises:
            aica_api.client.APIError: If the API call fails.
            ValueError: If the application name is invalid.
        """
        apps = self.__handle_errors(
            lambda: get_applications.sync(client=self.__get_client()),
            expect_empty=True,
        )
        app_id = next((app.id for app in apps.applications if app.name == name), None)
        if app_id is None:
            raise ValueError(f'No application with name "{name}" found in the application database')
        self.__handle_errors(
            lambda: set_current_application_by_id.sync(client=self.__get_client(), id=app_id),
            expect_empty=True,
        )

    def start_application(self) -> None:
        """
        Start the AICA application engine.

        Raises:
            aica_api.client.APIError: If the API call fails.
        """
        self.__start_application_transition(ApplicationLifecycleTransitionTransition.START)

    def stop_application(self) -> None:
        """
        Stop and reset the AICA application engine, removing all components and hardware interfaces.

        Raises:
            aica_api.client.APIError: If the API call fails.
        """
        self.__start_application_transition(ApplicationLifecycleTransitionTransition.STOP)

    def set_component_parameter(
        self,
        component: str,
        parameter: str,
        value: ValueParameterT,
    ) -> None:
        """
        Set a parameter on a component.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param component: The name of the component
        :param parameter: The name of the parameter
        :param value: The value of the parameter
        """
        self.__handle_errors(
            lambda: set_component_parameter.sync(
                client=self.__get_client(),
                body=SetComponentParameter(component=component, parameter=parameter, value=value),
            ),
            expect_empty=True,
        )

    def set_controller_parameter(
        self,
        hardware: str,
        controller: str,
        parameter: str,
        value: ValueParameterT,
    ) -> None:
        """
        Set a parameter on a controller.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller
        :param parameter: The name of the parameter
        :param value: The value of the parameter
        """
        self.__handle_errors(
            lambda: set_controller_parameter.sync(
                client=self.__get_client(),
                body=SetControllerParameter(hardware=hardware, controller=controller, parameter=parameter, value=value),
            ),
            expect_empty=True,
        )

    def set_lifecycle_transition(self, component: str, transition: LifecycleTransition) -> None:
        """
        Trigger a lifecycle transition on a component. The transition label must be one of the following:
        ['configure', 'activate', 'deactivate', 'cleanup', 'unconfigured_shutdown', 'inactive_shutdown',
        'acitve_shutdown']

        The transition will only be executed if the target is a lifecycle component and the current lifecycle state
        allows the requested transition.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param component: The name of the component
        :param transition: The lifecycle transition label
        """
        self.__handle_errors(
            lambda: trigger_component_transition.sync(
                client=self.__get_client(),
                body=ComponentLifecycleTransition(component=component, transition=transition),
            ),
            expect_empty=True,
        )

    def switch_controllers(
        self,
        hardware: str,
        activate: Union[None, List[str]] = None,
        deactivate: Union[None, List[str]] = None,
    ) -> None:
        """
        Activate and deactivate the controllers for a given hardware interface.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface
        :param activate: A list of controllers to activate
        :param deactivate: A list of controllers to deactivate
        """
        self.__handle_errors(
            lambda: switch_controllers.sync(
                client=self.__get_client(),
                body=SwitchControllersRequest(
                    hardware=hardware,
                    activate=activate if activate else [],
                    deactivate=deactivate if deactivate else [],
                ),
            ),
            expect_empty=True,
        )

    def unload_component(self, component: str) -> None:
        """
        Unload a component in the current application. If the component is not loaded, or if the component is not
        described in the application, nothing happens.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param component: The name of the component to unload
        """
        self.__handle_errors(
            lambda: unload_component.sync(
                client=self.__get_client(),
                body=ComponentReference(
                    component=component,
                ),
            ),
            expect_empty=True,
        )

    def unload_controller(self, hardware: str, controller: str) -> None:
        """
        Unload a controller for a given hardware interface. If the controller is not loaded, or if the controller
        is not listed in the hardware interface description, nothing happens.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface
        :param controller: The name of the controller to unload
        """
        self.__handle_errors(
            lambda: unload_controller.sync(
                client=self.__get_client(),
                body=LoadControllerRequest(hardware=hardware, controller=controller),
            ),
            expect_empty=True,
        )

    def unload_hardware(self, hardware: str) -> None:
        """
        Unload a hardware interface in the current application. If the hardware interface is not loaded, or if the
        interface is not described in the application, nothing happens.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface to unload
        """
        self.__handle_errors(
            lambda: unload_hardware.sync(
                client=self.__get_client(),
                body=HardwareReference(
                    hardware=hardware,
                ),
            ),
            expect_empty=True,
        )

    def get_application(self) -> CurrentApplication:
        """
        Get the currently set application

        Raises:
            aica_api.client.APIError: If the API call fails.
        """
        return self.__handle_errors(lambda: get_current_application.sync(client=self.__get_client()))

    def manage_sequence(self, sequence_name: str, transition: SequenceTransition) -> None:
        """
        Manage a sequence. The action label must be one of the following: ['start', 'restart', 'abort']

        The action will only be executed if the sequence exists and allows the requested action.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param sequence_name: The name of the sequence
        :param action: The sequence action label
        """
        self.__handle_errors(
            lambda: trigger_sequence_transition.sync(
                client=self.__get_client(),
                body=SequenceLifecycleTransition(
                    sequence=sequence_name,
                    transition=transition,
                ),
            ),
            expect_empty=True,
        )

    def wait_for_component(self, component: str, state: str, timeout: Union[None, int, float] = None) -> bool:
        """
        Wait for a component to be in a particular state. Components can be in any of the following states:
            ['unloaded', 'loaded', 'unconfigured', 'inactive', 'active', 'finalized']

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param component: The name of the component
        :param state: The state of the component to wait for
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the component is in the intended state before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[component]['state'] == state,
                url=self.__address,
                namespace='/v2/components',
                event='component_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    def wait_for_hardware(self, hardware: str, state: str, timeout: Union[None, int, float] = None) -> bool:
        """
        Wait for a hardware interface to be in a particular state. Hardware can be in any of the following states:
            ['unloaded', 'loaded']

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface
        :param state: The state of the hardware to wait for
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the hardware is in the intended state before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[hardware]['state'] == state,
                url=self.__address,
                namespace='/v2/hardware',
                event='hardware_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

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

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface responsible for the controller
        :param controller: The name of the controller
        :param state: The state of the controller to wait for
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the controller is in the intended state before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[hardware]['controllers'][controller]['state'] == state,
                url=self.__address,
                namespace='/v2/hardware',
                event='hardware_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    def wait_for_component_predicate(
        self, component: str, predicate: str, timeout: Union[None, int, float] = None
    ) -> bool:
        """
        Wait until a component predicate is true.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param component: The name of the component
        :param predicate: The name of the predicate
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the predicate is true before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[component]['predicates'][predicate],
                url=self.__address,
                namespace='/v2/components',
                event='component_data',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    def wait_for_controller_predicate(
        self,
        hardware: str,
        controller: str,
        predicate: str,
        timeout: Union[None, int, float] = None,
    ) -> bool:
        """
        Wait until a controller predicate is true.

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param hardware: The name of the hardware interface responsible for the controller
        :param controller: The name of the controller
        :param predicate: The name of the predicate
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the predicate is true before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[hardware]['controllers'][controller]['predicates'][predicate],
                url=self.__address,
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

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param condition: The name of the condition
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the condition is true before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[condition],
                url=self.__address,
                namespace='/v2/conditions',
                event='conditions',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )

    def wait_for_sequence(self, sequence: str, state: str, timeout=None) -> bool:
        """
        Wait for a sequence to be in a particular state. Sequences can be in any of the following states:
            ['active', 'inactive', 'aborted']

        Raises:
            aica_api.client.APIError: If the API call fails.

        :param sequence: The name of the sequence
        :param state: The state of the sequence to wait for
        :param timeout: Timeout duration in seconds. If set to None, block indefinitely
        :return: True if the condition is true before the timeout duration, False otherwise
        """
        return (
            read_until(
                lambda data: data[sequence]['state'] == state,
                url=self.__address,
                namespace='/v2/sequences',
                event='sequences',
                timeout=timeout,
                auth=self._sio_auth(),
            )
            is not None
        )
