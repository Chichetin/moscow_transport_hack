"""Run from the repository root: .venv/bin/python tools/stand/run_stand.py out/stand/<bag>"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

if __name__ == '__main__':
    from tram_stand.cli import main
    sys.exit(main())
