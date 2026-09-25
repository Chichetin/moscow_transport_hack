import sys

from .cli import main

if __name__ == '__main__':   # process-pool workers re-import __main__ on Windows (spawn)
    sys.exit(main())
