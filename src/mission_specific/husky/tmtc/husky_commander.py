__author__ = "Amaan Javed"
__copyright__ = "Copyright 2026, JAOPS"
__license__ = "BSD-3-Clause"
__version__ = "3.0.0"
__status__ = "development"

import omni.kit.app

from src.mission_specific.husky.tmtc.enums import HuskyYamcsArguments, HuskyCameraResolution
from src.mission_specific.husky.tmtc.camera_handler import CameraViewType, HuskyCameraHandler
from src.mission_specific.husky.tmtc.transmitter import HuskyTransmitter
from src.mission_specific.husky.subsystems.husky_robot_enums import GoNogoState, ObcState
from src.subsystems.device import CommonDevice, HealthState, PowerState
from src.tmtc.intervals_handler import IntervalName


class HuskyCommander:
    """
    Commander for the Husky UGV.

    Contains the rover-specific logic behind each Yamcs command.  Methods here
    are registered as callbacks inside HuskyController.setup_command_callbacks().

    Husky-specific capabilities vs. Pragyaan:
      * Differential-drive only (drive_straight / drive_turn).
      * Onboard camera (RGBA + depth capture, streaming on/off).
      * Power control for camera and motor controller.
      * GO / NOGO gate.
      * No solar panel, no APXS, no neutron spectrometer.
    """

    def __init__(
        self,
        robot,
        intervals,
        camera_handler: HuskyCameraHandler,
        transmitter: HuskyTransmitter,
        drive_handler,
        obc_handler,
        intervals_handler,
    ):
        self._robot = robot
        self._intervals = intervals
        self._camera_handler = camera_handler
        self._transmitter = transmitter
        self._drive_handler = drive_handler
        self._obc_handler = obc_handler
        self._intervals_handler = intervals_handler

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def drive_straight(self, linear_velocity, distance):
        self._drive_handler.drive_robot_straight(linear_velocity, distance)

    def drive_turn(self, angular_velocity, angle):
        self._drive_handler.drive_robot_turn(angular_velocity, angle)

    # ------------------------------------------------------------------
    # Camera commands
    # ------------------------------------------------------------------

    def handle_high_res_capture(self):
        if self._robot.subsystems.get_device_power_state(CommonDevice.CAMERA) == PowerState.ON:
            self._camera_handler.transmit_camera_view(
                HuskyCameraHandler.BUCKET_ONCOMMAND,
                HuskyCameraResolution.HIGH.value,
                CameraViewType.RGBA,
            )
            self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def handle_depth_capture(self):
        if self._robot.subsystems.get_device_power_state(CommonDevice.CAMERA) == PowerState.ON:
            self._camera_handler.transmit_camera_view(
                HuskyCameraHandler.BUCKET_DEPTH,
                HuskyCameraResolution.HIGH.value,
                CameraViewType.DEPTH,
            )
            self._obc_handler.set_obc_state(ObcState.CAMERA, 10)

    def set_activity_of_camera_streaming(self, action: str):
        if action == HuskyYamcsArguments.STOP.value:
            self._intervals_handler.remove_interval(IntervalName.CAMERA_STREAMING.value)
        elif action == HuskyYamcsArguments.START.value:
            if not self._intervals_handler.does_exist(IntervalName.CAMERA_STREAMING.value):
                self._intervals_handler.add_new_interval(
                    name=IntervalName.CAMERA_STREAMING.value,
                    seconds=self._intervals["camera_streaming"],
                    is_repeating=True,
                    execute_immediately=False,
                    function=self._camera_handler.transmit_camera_view,
                    f_args=(
                        HuskyCameraHandler.BUCKET_STREAMING,
                        HuskyCameraResolution.LOW.value,
                    ),
                )
        else:
            print("HuskyCommander.set_activity_of_camera_streaming: unknown action:", action)

    # ------------------------------------------------------------------
    # Power / electronics commands
    # ------------------------------------------------------------------

    def handle_electronics_on_off(self, electronics: str, new_state: str):
        if new_state not in [PowerState.ON.value, PowerState.OFF.value]:
            print("HuskyCommander: unknown PowerState:", new_state)
            return

        new_state = PowerState[new_state]
        self._robot.subsystems.set_device_power_state(electronics, new_state)

        if electronics == CommonDevice.CAMERA:
            action = HuskyYamcsArguments.START.value if new_state == PowerState.ON else HuskyYamcsArguments.STOP.value
            self.set_activity_of_camera_streaming(action)
        elif electronics == CommonDevice.MOTOR_CONTROLLER:
            if new_state == PowerState.OFF:
                self._drive_handler.stop_robot()
                self._robot.subsystems.set_device_health_state(
                    CommonDevice.MOTOR_CONTROLLER, HealthState.NOMINAL
                )

    # ------------------------------------------------------------------
    # System commands
    # ------------------------------------------------------------------

    def handle_go_nogo(self, decision: str):
        if decision not in [GoNogoState.GO.name, GoNogoState.NOGO.name]:
            print("HuskyCommander: unknown GO/NOGO decision:", decision)
            return

        decision = GoNogoState[decision]
        self._robot.subsystems.set_go_nogo_state(decision)

        if decision == GoNogoState.NOGO:
            self._drive_handler.stop_robot()

    def inject_fault(self):
        self._robot.subsystems.set_device_health_state(
            CommonDevice.MOTOR_CONTROLLER, HealthState.FAULT
        )
        self._drive_handler.stop_robot()

    def handle_battery_perc_change(self, battery_percentage: int):
        self._robot.subsystems.set_battery_perc(battery_percentage)
