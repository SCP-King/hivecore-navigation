"""One navigation task per GUI process, shared across page/module executions."""
from dataclasses import dataclass,field


@dataclass
class NavigationSession:
    active: bool=False
    cancel: bool=False
    pause: bool=False
    feedback: dict=field(default_factory=dict)
    binding: object=None
    context: object=None
    task: object=None
    pending_spawn: object=None
    mode: str='地图选点移动'
    switching: bool=False


SESSION=NavigationSession()
