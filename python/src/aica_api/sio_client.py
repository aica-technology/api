import time
from typing import Callable, Union

import socketio
from socketio.exceptions import ConnectionError, TimeoutError


def read_once(url: str = 'http://0.0.0.0:5000', namespace: str = '/', event: str = '*',
              timeout: Union[None, int, float] = 5) -> Union[None, dict]:
    """
    Listen for and return the first Socket.IO event on a specified URL and namespace within a time limited period

    :param url: The Socket.IO server URL
    :param namespace: The Socket.IO namespace (channel)
    :param event: The Socket.IO event name. By default, all events are accepted with a wildcard
    :param timeout: The timeout in seconds to listen for an event. If set to None, block indefinitely
    :return: The received event data, or None if the connection or event listener timed out
    """
    return read_until(lambda data: True, url=url, namespace=namespace, event=event, timeout=timeout)


def read_until(callback: Callable[[dict], bool], url: str = 'http://0.0.0.0:5000', namespace: str = '/',
               event: str = '*', timeout: Union[None, int, float] = 5) -> Union[None, dict]:
    """
    Listen for and return the first Socket.IO event that validates against a callback function on a specified URL
    and namespace within a time limited period

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

    with socketio.SimpleClient() as sio:
        try:
            sio.connect(url, namespace=namespace, wait_timeout=timeout)
        except ConnectionError:
            print(f"Could not connect!")
            return None

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
