"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
HEIGHT_TOLERANCE = 0.5  # meters
ANGLE_TOLERANCE_DEG = 5
ANGLE_TOLERANCE_RAD = math.radians(ANGLE_TOLERANCE_DEG)
Z_SPEED = 1  # m/s
TURNING_SPEED = 5  # deg/s


class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        try:
            return True, cls(cls.__private_key, connection, target, local_logger)
        except Exception as e:
            local_logger.error(f"Failed to create Command: {e}", True)
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        # Do any intializiation here
        self._connection = connection
        self._target = target
        self._logger = local_logger
        self._velocity_sum_x = 0.0
        self._velocity_sum_y = 0.0
        self._velocity_sum_z = 0.0
        self._velocity_count = 0

    def run(
        self,
        telemetry_data: telemetry.TelemetryData,
    ) -> list[str]:
        """
        Make a decision based on received telemetry data.
        Returns list of strings to send to main (e.g. "CHANGE ALTITUDE: 1.0").
        """
        # Log average velocity for this trip so far
        output_strings: list[str] = []

        if (
            telemetry_data.x_velocity is not None
            and telemetry_data.y_velocity is not None
            and telemetry_data.z_velocity is not None
        ):
            self._velocity_sum_x += telemetry_data.x_velocity
            self._velocity_sum_y += telemetry_data.y_velocity
            self._velocity_sum_z += telemetry_data.z_velocity
            self._velocity_count += 1
            if self._velocity_count > 0:
                avg_vx = self._velocity_sum_x / self._velocity_count
                avg_vy = self._velocity_sum_y / self._velocity_count
                avg_vz = self._velocity_sum_z / self._velocity_count
                self._logger.info(
                    f"Average velocity: ({avg_vx}, {avg_vy}, {avg_vz}) m/s",
                    True,
                )

        # Use COMMAND_LONG (76) message, assume the target_system=1 and target_componenet=0
        # The appropriate commands to use are instructed below
        if (
            telemetry_data.z is not None
            and abs(telemetry_data.z - self._target.z) > HEIGHT_TOLERANCE
        ):
            # Adjust height using the comand MAV_CMD_CONDITION_CHANGE_ALT (113)
            # String to return to main: "CHANGE_ALTITUDE: {amount you changed it by, delta height in meters}"
            delta_z = self._target.z - telemetry_data.z
            try:
                self._connection.mav.command_long_send(
                    1,  # target_system
                    0,  # target_component
                    mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                    0,  # confirmation
                    Z_SPEED,  # param1: ascent/descent speed
                    0,  # param2
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    self._target.z,  # param7: target altitude
                )
                output_strings.append(f"CHANGE ALTITUDE: {delta_z}")
            except Exception as e:
                self._logger.error(f"Failed to send altitude command: {e}", True)
        elif (
            telemetry_data.x is not None
            and telemetry_data.y is not None
            and telemetry_data.yaw is not None
        ):
            # Adjust direction (yaw) using MAV_CMD_CONDITION_YAW (115). Must use relative angle to current state
            # String to return to main: "CHANGING_YAW: {degree you changed it by in range [-180, 180]}"
            # Positive angle is counter-clockwise as in a right handed system
            desired_yaw = math.atan2(
                self._target.y - telemetry_data.y,
                self._target.x - telemetry_data.x,
            )
            delta_yaw = desired_yaw - telemetry_data.yaw
            # Normalize to [-pi, pi]
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi

            if abs(delta_yaw) > ANGLE_TOLERANCE_RAD:
                delta_yaw_deg = math.degrees(delta_yaw)
                delta_yaw_deg = max(-180, min(180, delta_yaw_deg))
                try:
                    self._connection.mav.command_long_send(
                        1,  # target_system
                        0,  # target_component
                        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                        0,  # confirmation
                        delta_yaw_deg,  # param1: target angle (degrees)
                        TURNING_SPEED,  # param2: speed deg/s
                        0,  # param3
                        1,  # param4: relative (1)
                        0,  # param5
                        0,  # param6
                        0,  # param7
                    )
                    output_strings.append(f"CHANGE YAW: {delta_yaw_deg}")
                except Exception as e:
                    self._logger.error(f"Failed to send yaw command: {e}", True)

        return output_strings


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
