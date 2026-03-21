"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import heartbeat_receiver
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_receiver_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    connection: MAVLink connection to the drone.
    output_queue: Queue to report connection status ("Connected" or "Disconnected") to main.
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

    local_logger.info("Heartbeat receiver logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (heartbeat_receiver.HeartbeatReceiver)
    result, heartbeat_receiver_instance = heartbeat_receiver.HeartbeatReceiver.create(
        connection, local_logger
    )
    if not result:
        local_logger.error("Failed to create HeartbeatReceiver", True)
        return

    # Main loop: receive heartbeats and report status every second
    while not controller.is_exit_requested():
        controller.check_pause()

        continue_loop, state = heartbeat_receiver_instance.run()
        if not continue_loop:
            break
        output_queue.queue.put(state)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
