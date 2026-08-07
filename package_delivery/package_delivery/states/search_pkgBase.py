import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from nectar.control import MavrosDrone, MoveReference
from package_delivery.constants import Config

# Estado para futura implementação de coleta autonoma!
class SearchPkgBase(State):
    pass