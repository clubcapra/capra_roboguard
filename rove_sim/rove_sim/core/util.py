"""Low-level helpers."""
from __future__ import annotations

import contextlib
import os
import sys
import threading
import time


def die_with_parent(poll_s: float = 2.0) -> None:
    """Self-terminate when our parent process dies.

    The fleet launchers (sim_fleet.sh / gui_fleet.sh) start these worker
    processes as children of a bash script. If that script is killed with
    SIGKILL its EXIT trap never runs, so the workers get re-parented (to PID 1
    or a subreaper) and leak -- orphaned cameras/lidar/mediamtx pile up across
    restarts. This watchdog polls our parent PID and hard-exits the moment it
    changes, guaranteeing no orphans regardless of how the parent goes away.
    """
    ppid0 = os.getppid()

    def _watch():
        while True:
            time.sleep(poll_s)
            if os.getppid() != ppid0:
                os._exit(0)

    threading.Thread(target=_watch, name="die-with-parent", daemon=True).start()


@contextlib.contextmanager
def suppressed_fds(enabled: bool = True):
    """Silence C-level stdout/stderr (e.g. pybullet's b3Warning URDF spam).

    Python-level prints are flushed first; then fds 1/2 are redirected to
    /dev/null for the duration. Set enabled=False to pass through (debugging).
    """
    if not enabled:
        yield
        return
    sys.stdout.flush(); sys.stderr.flush()
    devnull = os.open(os.devnull, os.O_WRONLY)
    saved = (os.dup(1), os.dup(2))
    try:
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        os.dup2(saved[0], 1)
        os.dup2(saved[1], 2)
        os.close(saved[0]); os.close(saved[1]); os.close(devnull)
