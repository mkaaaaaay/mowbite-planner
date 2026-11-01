#!/usr/bin/env python3
"""The planner's size rules for what's built anew: a module MAX_MODULE lines at most, a function MAX_FUNCTION.

  sizes.py [package dir]   one line a module or function over, exit 1 then

The core the planner keeps (KEPT) and the old parts still in use (OLD) aren't held to them.
"""
import ast
import os
import sys

MAX_MODULE = 600
MAX_FUNCTION = 80
KEPT = {'__init__', 'angles', 'bodyfit', 'config', 'drive', 'export', 'footprint', 'geometry', 'maneuvers', 'mapfile',
        'mathutil', 'mqtt_service', 'node', 'report', 'space', 'sweep', 'transit'}
OLD = {'lanes'}


def over(path: str):
    """(where, what) of each rule the module at path breaks."""
    with open(path) as f:
        text = f.read()
    lines = text.count('\n') + (not text.endswith('\n'))
    if lines > MAX_MODULE:
        yield path, f'{lines} lines (at most {MAX_MODULE})'
    for node in ast.walk(ast.parse(text, path)):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            n = node.end_lineno - node.lineno + 1
            if n > MAX_FUNCTION:
                yield f'{path}:{node.lineno}', f'{node.name} {n} lines (at most {MAX_FUNCTION})'


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '..', 'mowbite_planner')
    names = sorted(f[:-3] for f in os.listdir(pkg) if f.endswith('.py') and f[:-3] not in KEPT | OLD)
    bad = [hit for name in names for hit in over(os.path.join(pkg, name + '.py'))]
    for where, what in bad:
        print(where, what)
    print(f'{len(names)} modules, {len(bad)} over')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
