# Пути импорта для pytest из корня (pytest 6 в ROS Humble не знает ini-опцию pythonpath).
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
for sub in ('src/tram_odometry_core', 'tools/eval', 'tools/stand'):
    sys.path.insert(0, str(ROOT / sub))
