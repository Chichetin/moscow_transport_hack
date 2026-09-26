# Пути импорта для pytest из корня (pytest 6 в ROS Humble не знает ini-опцию pythonpath).
import os
import sys
from pathlib import Path

# Однопоточный BLAS (#68): numpy-колёса несут многопоточный OpenBLAS с DYNAMIC_ARCH. На CI один
# раз np.linalg.solve в notebooks/identification.fit_curve вернул узел 0,038 в стороне от
# однопоточного результата -- примерно на 11 порядков больше, чем шум округления при числе
# обусловленности матрицы ~1e3 (~1e-13). Причина не установлена: гонка потоков в самом OpenBLAS
# или баг кернеля DYNAMIC_ARCH под конкретный CPU -- какая из двух, не выяснено; однопоточность
# лечит первую гипотезу дёшево, но не обязательно вторую (см. ревью PR #95, D-060: если
# повторится и с этой защитой, следующий шаг -- numpy.show_config() и lscpu из упавшего прогона
# CI). Явно заданный снаружи вариант этих переменных не трогается (`setdefault`) -- если он не
# 1, тест-страж ниже честно падает, это не новый флаки. Ставится здесь, до первого импорта
# numpy любым тестом в сессии.
_NUMPY_ALREADY_IMPORTED = 'numpy' in sys.modules   # too late to pin if True -- guarded by #68's test
for _var in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ.setdefault(_var, '1')

ROOT = Path(__file__).resolve().parent
for sub in ('src/tram_odometry_core', 'tools/eval', 'tools/stand', 'tools/submission', 'tools/survey'):
    sys.path.insert(0, str(ROOT / sub))
