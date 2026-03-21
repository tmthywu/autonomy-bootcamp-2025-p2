"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


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
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatReceiver] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        try:
            return True, cls(cls.__private_key, connection, local_logger)
        except (OSError, TimeoutError, TypeError, AttributeError) as e:
            local_logger.error(f"Failed to create HeartbeatReceiver: {e}", True)
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        self._connection = connection
        self._logger = local_logger
        self._missed_count = 0
        self._connected = False

    def run(self) -> tuple[bool, str]:
        """
        Attempt to receive a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        Every second, returns current state ("Connected" or "Disconnected").
        Returns (True, state) to continue, (False, state) to stop (e.g. on sentinel).
        """
        try:
            msg = self._connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        except (OSError, TimeoutError) as e:
            self._logger.error(f"Error receiving heartbeat: {e}", True)
            self._missed_count += 1
            if self._missed_count >= DISCONNECT_THRESHOLD:
                self._connected = False
            state = "Connected" if self._connected else "Disconnected"
            return True, state

        if msg and msg.get_type() == "HEARTBEAT":
            self._missed_count = 0
            self._connected = True
        else:
            self._missed_count += 1
            self._logger.warning("Missed heartbeat", True)
            if self._missed_count >= DISCONNECT_THRESHOLD:
                self._connected = False

        state = "Connected" if self._connected else "Disconnected"
        return True, state


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
