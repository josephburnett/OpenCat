import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parent / "gen", HERE.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
