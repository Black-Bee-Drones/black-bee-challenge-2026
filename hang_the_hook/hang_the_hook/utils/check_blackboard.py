from yasmin import Blackboard, YASMIN_LOG_ERROR
from typing import Tuple, List

def blackboard_check(blackboard: Blackboard, args: Tuple[str, ...]) -> bool:
    """
    Checks instances in Blackboard

    Parameters
    ----------
    blackboard: Yasmin blackboard to be checked
    args: keywords to search in blackboard
    log: select log option | Yasmin log, Terminal

    Returns
    -------
    bool
        - If any of the args is not in the given blackboard, returns False
    """

    result_parse: List = [False] * len(args)

    for i in range(len(args)):
        if args[i] not in blackboard:
            YASMIN_LOG_ERROR(f'{args[i].capitalize()} not in Blackboard')
        else:
            result_parse[i] = True

    for i in range(len(result_parse)):
        if not result_parse[i]:
            return False

    return True
