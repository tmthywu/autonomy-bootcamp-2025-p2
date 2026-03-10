"""
Heartbeat sending logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatSender:
    """
    HeartbeatSender class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatSender] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatSender object.
        """
        try:
            return True, cls(cls.__private_key, connection, local_logger)
        except Exception as e:
            local_logger.error(f"Failed to create HeartbeatSender: {e}", True)
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ):
        assert key is HeartbeatSender.__private_key, "Use create() method"

        self._connection = connection
        self._logger = local_logger

    def run(self) -> bool:
        """
        Attempt to send a heartbeat message.
        Returns True on success, False on failure.
        """
        try:
            self._connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            return True
        except Exception as e:
            self._logger.error(f"Failed to send heartbeat: {e}", True)
            return False


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
