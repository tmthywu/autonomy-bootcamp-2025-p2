"""
Telemtry worker that gathers GPS data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import telemetry
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def telemetry_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    connection: MAVLink connection to the drone.
    output_queue: Queue to send TelemetryData to the Command worker.
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

    local_logger.info("Telemetry logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (telemetry.Telemetry)
    result, telemetry_instance = telemetry.Telemetry.create(connection, local_logger)
    if not result:
        local_logger.error("Failed to create Telemetry", True)
        return

    # Main loop: gather telemetry and send to Command worker
    while not controller.is_exit_requested():
        controller.check_pause()

        success, telemetry_data = telemetry_instance.run()
        if success:
            output_queue.queue.put(telemetry_data)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
