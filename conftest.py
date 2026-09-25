# Пути импорта для pytest из корня (pytest 6 в ROS Humble не знает ini-опцию pythonpath).
import os
import sys
from pathlib import Path

# Однопоточный BLAS (#68): numpy-колёса несут OpenBLAS, и его многопоточная редукция даёт
# разный порядок суммирования на раннерах CI с разным числом ядер -- один и тот же код и seed
# дают чуть разный np.linalg.solve в notebooks/identification и изредка выходят за допуск
# теста подгонки. Ставится здесь, до первого импорта numpy любым тестом в сессии.
for _var in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ.setdefault(_var, '1')

ROOT = Path(__file__).resolve().parent
for sub in ('src/tram_odometry_core', 'tools/eval', 'tools/stand', 'tools/submission'):
    sys.path.insert(0, str(ROOT / sub))
