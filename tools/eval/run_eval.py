"""Run tram_eval from the repository root: .venv/bin/python tools/eval/run_eval.py --split quick"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from tram_eval.cli import main  # noqa: E402

sys.exit(main())
