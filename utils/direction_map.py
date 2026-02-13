from __future__ import annotations

from typing import Mapping

from lerobot.configs.types import PipelineFeatureType, PolicyFeature
from lerobot.processor import ProcessorStepRegistry, RobotActionProcessorStep
from lerobot.processor.core import RobotAction


@ProcessorStepRegistry.register("direction_map_processor")
class DirectionMapProcessorStep(RobotActionProcessorStep):
    """
    Processor step to apply sign flips to robot actions based on motor base names.
    Intended for teleop/record pipelines so that mapping stays config-driven.
    """

    def __init__(self, direction_map: Mapping[str, float] | None = None):
        self.direction_map = dict(direction_map or {})

    def action(self, action: RobotAction) -> RobotAction:
        return {
            key: (val * self.direction_map.get(key.split(".")[0], 1.0))
            for key, val in action.items()
        }

    def get_config(self) -> dict[str, Mapping[str, float]]:
        return {"direction_map": self.direction_map}

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features

def apply_direction_map(action: Mapping[str, float], direction_map: Mapping[str, float] | None) -> dict[str, float]:
    """
    Apply sign flips to actions based on the motor base name (the part before '.pos' etc.).
    If no direction_map is provided, the action is returned as-is.
    """
    if not direction_map:
        return dict(action)

    return {
        key: (val * direction_map.get(key.split(".")[0], 1.0))
        for key, val in action.items()
    }
