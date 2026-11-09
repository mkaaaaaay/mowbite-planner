"""The processes of their own that plan (set_workers): an area's plans round its slots by the edge at once, the plans of
callers on threads of their own at once, the same plans as in the caller's. Started fresh (spawn), not forked from a
process with threads of its own. A planning stopped from outside (progress.stoppable) ends the ones at its plans."""

import functools
import multiprocessing
import threading
from concurrent.futures import Future, ProcessPoolExecutor, TimeoutError as FutureTimeout
from concurrent.futures.process import BrokenProcessPool
from typing import Any, Callable, Dict, Optional

from .progress import Stopped, check

_WORKERS = 0
_POOL: Optional[ProcessPoolExecutor] = None
_POOL_LOCK = threading.Lock()
# the plans handed to the pool and not done yet (a stopped planning's: the processes end, _stop_pool), s between two
# looks whether a planning waiting for one is to stop
_HANDED: set = set()
STOP_POLL = 0.2


def set_workers(n: int) -> None:
    """Plans in up to n processes of their own from now on, 0 (the default): all in this one."""
    global _WORKERS, _POOL
    with _POOL_LOCK:
        if _POOL is not None and int(n) != _WORKERS:
            _POOL.shutdown()  # (it ends what it's at: not waiting, Python 3.8 complains when the pipes close)
            _POOL = None
        _WORKERS = max(0, int(n))


def _pool() -> Optional[ProcessPoolExecutor]:
    global _POOL
    with _POOL_LOCK:
        if _POOL is None and _WORKERS > 0:
            _POOL = ProcessPoolExecutor(_WORKERS, mp_context=multiprocessing.get_context('spawn'))
        return _POOL


def _stop_pool() -> None:
    """A planning stopped: where the pool is still at its plans, its processes end at once (a new one when next
    asked)."""
    global _POOL
    with _POOL_LOCK:
        pool = _POOL
        if pool is None or all(f.done() for f in list(_HANDED)):
            return
        _POOL = None
    end = getattr(pool, 'terminate_workers', None)  # (Python 3.14)
    if end is not None:
        end()
    else:
        for process in list((getattr(pool, '_processes', None) or {}).values()):
            process.terminate()
    pool.shutdown()  # (at once, they're gone; not waiting for its thread, Python 3.8's fails on the closed pipe)


def later(fn: Callable, args: tuple, kw: Dict[str, Any]) -> 'Future':
    """fn(args, kw) in a process of the pool where there's one (a future of it), else at once."""
    pool = _pool()
    if pool is not None:
        try:
            future = pool.submit(fn, args, kw)
            _HANDED.add(future)
            future.add_done_callback(_HANDED.discard)
            return future
        except (BrokenProcessPool, RuntimeError):  # (one of them died, or it's shut down: in this one)
            set_workers(_WORKERS)
    check()
    done: Future = Future()
    try:
        done.set_result(fn(args, kw))
    except BaseException as e:  # noqa: BLE001 - handed on by result()
        done.set_exception(e)
    return done


def result(future: 'Future', fn: Callable, args: tuple, kw: Dict[str, Any]) -> Any:
    """later's result, Stopped where the planning waiting for it is to stop."""
    try:
        while True:
            try:
                return future.result(STOP_POLL)
            except FutureTimeout:
                check()
    except BrokenProcessPool:  # (a process of the pool died, say out of memory: in this one, the pool anew)
        check()
        with _POOL_LOCK:
            global _POOL
            _POOL = None
        return fn(args, kw)


def stops_pool(fn):
    """fn, the pool's processes ended where it's stopped while they're at its plans (_stop_pool)."""
    @functools.wraps(fn)
    def stoppable(*args, **kw):
        try:
            return fn(*args, **kw)
        except Stopped:
            _stop_pool()
            raise
    return stoppable
