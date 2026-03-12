"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger
from utilities.workers import queue_proxy_wrapper


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
DISCONNECT_THRESHOLD = 5


class HeartbeatReceiver:
    """
    HeartbeatReceiver class to receive heartbeats from the drone and report connection status.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        output_queue: queue_proxy_wrapper.QueueProxyWrapper,
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatReceiver] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        try:
            return True, cls(cls.__private_key, connection, output_queue, local_logger)
        except Exception as e:
            local_logger.error(f"Failed to create HeartbeatReceiver: {e}", True)
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        output_queue: queue_proxy_wrapper.QueueProxyWrapper,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        self._connection = connection
        self._output_queue = output_queue
        self._logger = local_logger
        self._missed_count = 0
        self._connected = False

    def run(self) -> bool:
        """
        Attempt to receive a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        Every second, reports current state ("Connected" or "Disconnected") to the output queue.
        Returns True to continue, False to stop (e.g. on sentinel).
        """
        try:
            msg = self._connection.recv_match(
                type="HEARTBEAT", blocking=True, timeout=1
            )
        except Exception as e:
            self._logger.error(f"Error receiving heartbeat: {e}", True)
            self._missed_count += 1
            if self._missed_count >= DISCONNECT_THRESHOLD:
                self._connected = False
            state = "Connected" if self._connected else "Disconnected"
            self._output_queue.queue.put(state)
            return True

        if msg and msg.get_type() == "HEARTBEAT":
            self._missed_count = 0
            self._connected = True
        else:
            self._missed_count += 1
            self._logger.warning("Missed heartbeat", True)
            if self._missed_count >= DISCONNECT_THRESHOLD:
                self._connected = False

        state = "Connected" if self._connected else "Disconnected"
        self._output_queue.queue.put(state)
        return True


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
