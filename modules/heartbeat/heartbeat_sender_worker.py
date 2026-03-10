"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib
import time

from pymavlink import mavutil

from ..common.modules.logger import logger
from utilities.workers import worker_controller
from . import heartbeat_sender



# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_sender_worker(
    connection: mavutil.mavfile,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    connection: MAVLink connection to the drone.
    controller: How the main process communicates to this worker (exit/pause).
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (heartbeat_sender.HeartbeatSender)
    result, heartbeat_sender_instance = heartbeat_sender.HeartbeatSender.create(
        connection, local_logger
    )
    if not result or heartbeat_sender_instance is None:
        local_logger.error("Failed to create HeartbeatSender", True)
        return

    # Main loop: send heartbeat once per second
    while not controller.is_exit_requested():
        controller.check_pause()

        heartbeat_sender_instance.run()
        time.sleep(1)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
