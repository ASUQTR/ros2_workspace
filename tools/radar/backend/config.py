import os

ROSBRIDGE_HOST = os.environ.get("RADAR_ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.environ.get("RADAR_ROSBRIDGE_PORT", "9090"))

# Services (trigger the real hardware; also callable by other components,
# e.g. a future playback control node).
SCAN_SERVICE = "/sonar/scan"
SCAN_SERVICE_TYPE = "sub_interfaces/srv/SonarScan"
LOCALIZATION_SERVICE = "/get_sonar_position"
LOCALIZATION_SERVICE_TYPE = "sub_interfaces/srv/GetSonarPosition"

# Topics — sonar_node and sonar_localization mirror their own service
# response onto these right before returning it to whoever called them.
SCAN_TOPIC = "/service/scan"
SCAN_TOPIC_TYPE = "sub_interfaces/msg/SonarScanResult"
LOCALIZATION_TOPIC = "/service/localization"
LOCALIZATION_TOPIC_TYPE = "sub_interfaces/msg/SonarPositionResult"

DEFAULT_SCAN_START_ANGLE = 0
DEFAULT_SCAN_STOP_ANGLE = 359
DEFAULT_SCAN_RANGE_M = 20

SERVICE_CALL_TIMEOUT_S = 30.0
