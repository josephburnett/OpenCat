"""python -m kilo.vbody: same as `kilo vbody`."""

import sys

from kilo.cli import main

sys.exit(main(["vbody", *sys.argv[1:]]))
