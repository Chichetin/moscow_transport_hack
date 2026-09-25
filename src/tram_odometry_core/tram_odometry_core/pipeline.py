"""Single entry point used by the ROS node and tools/eval (docs/contracts.md, section 2)."""
from typing import Any, Optional

from .types import Estimate, Params


class Odometry:
    def __init__(self, params: Params, route=None):
        self.params = params
        self.route = route

    def step(self, raw: Any) -> Optional[Estimate]:
        """Consume one raw input; None means the input was dropped, nothing to publish."""
        return None
