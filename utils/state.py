from enum import Enum, unique

@unique
class State(Enum):
    idle = 0
    start = 1
    finished = 2
    failed = 3
    waiting = 4 
    end_waiting = 5
    end_current_point = 6
