import time
import urllib.parse
from typing import Callable, Optional, Union

import socketio
from socketio.exceptions import ConnectionError, TimeoutError

from aica_api.errors import APIError


def read_once(
    url: str = 'http://0.0.0.0:8080/api',
    namespace: str = '/v3/ws',
    event: str = '*',
    timeout: Union[None, int, float] = 5,
    auth: Optional[str] = None,
) -> Union[None, dict]:
    """
    Listen for and return the first Socket.IO event on a specified URL and namespace within a time limited period

    Raises:
        aica_api.client.APIError: If the API call fails.

    :param url: The Socket.IO server URL
    :param namespace: The Socket.IO namespace (channel)
    :param event: The Socket.IO event name. By default, all events are accepted with a wildcard
    :param timeout: The timeout in seconds to listen for an event. If set to None, block indefinitely
    :return: The received event data, or None if the connection or event listener timed out
    """
    return read_until(
        lambda data: True,
        url=url,
        namespace=namespace,
        event=event,
        timeout=timeout,
        auth=auth,
    )


def read_until(
    callback: Callable[[dict], bool],
    url: str = 'http://0.0.0.0:8080/api',
    namespace: str = '/v3/ws',
    event: str = '*',
    timeout: Union[None, int, float] = 5,
    auth: Optional[str] = None,
) -> Union[None, dict]:
    """
    Listen for and return the first Socket.IO event that validates against a callback function on a specified URL
    and namespace within a time limited period

    Raises:
        aica_api.client.APIError: If the API call fails.

    :param callback: A data callback function taking a single dict argument and returning true or false.
        KeyErrors are automatically suppressed. For example:
            def user_callback(data: dict) -> bool:
                return data['foo'] == 'bar'
    :param url: The Socket.IO server URL
    :param namespace: The Socket.IO namespace (channel)
    :param event: The Socket.IO event name. By default, all events are accepted with a wildcard
    :param timeout: The timeout in seconds to listen for an event. If set to None, block indefinitely
    :return: The received event data, or None if the connection or event listener timed out
    """

    base_path = urllib.parse.urlparse(url).path

    with socketio.SimpleClient() as sio:
        try:
            sio.connect(
                url,
                # socket.io just wipes the path in the url completely
                # https://github.com/miguelgrinberg/python-engineio/blob/58c37d48819a382c2c553365b1e1dbb10d382ed3/src/engineio/base_client.py#L148
                socketio_path=f'{base_path}/v3/ws',
                namespace=namespace,
                wait_timeout=timeout,
                auth={'auth': f'Bearer {auth}'},
            )
        except ConnectionError as e:
            raise APIError('socket.io failed to connect') from e

        start_time = time.time()
        while True:
            try:
                if timeout is None:
                    received = sio.receive()
                else:
                    elapsed = time.time() - start_time
                    if elapsed >= timeout:
                        raise TimeoutError
                    received = sio.receive(timeout=timeout - elapsed)
            except TimeoutError:
                break
            else:
                if event == '*' or event == received[0]:
                    try:
                        data = received[1]
                        if callback(data):
                            return data
                    except KeyError:
                        # invalid key access in the callback function, ignore
                        pass
