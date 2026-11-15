"""GUI compatibility import for the shared map-floor estimator."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/n01_live_navigation'))
from n01_live_navigation.ground_plane import GroundPlane, estimate_ground
