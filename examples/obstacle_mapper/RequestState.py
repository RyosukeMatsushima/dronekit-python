
from enum import Enum

class RequestState(Enum):
    GO_TO_CLICK_POINT = 1
    GO_TO_HOME = 2
    TAKE_OFF = 3
    LAND = 4
    STOP_DRONE = 5