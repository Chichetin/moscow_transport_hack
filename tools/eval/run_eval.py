"""Run tram_eval from the repository root: .venv/bin/python tools/eval/run_eval.py --split quick"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

if __name__ == '__main__':   # process-pool workers re-import this file on Windows (spawn)
    from tram_eval.cli import main
    sys.exit(main())
