#!/usr/bin/env python3
"""Plans an area offline and draws it with matplotlib, no ROS needed (only shapely and matplotlib).

    python3 tools/plot_coverage.py                      # demo garden
    python3 tools/plot_coverage.py --radius 0.3 --passes 1 --save plan.png
    python3 tools/plot_coverage.py --lane-order skip --no-reverse --coverage    # for a forwards-only controller
    python3 tools/plot_coverage.py --json area.json     # {"boundary": [[x, y], ...], "obstacles": [[[x, y], ...], ...]}
    python3 tools/plot_coverage.py --map map.json --area "Front lawn"   # an area of OpenMower's map.json
    python3 tools/plot_coverage.py --pattern concentric --perimeter-order last --end 0 3
    python3 tools/plot_coverage.py --angle-strategy optimal --angle-range 20 60 --turn-types bulb,pivot

Outline black, obstacles red, drivable areas grey, perimeter loops blue, lanes green, turns orange (backing
up dashed), transits violet dotted, arrows show the heading of the poses.
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner import PlannerConfig, PlanningError, plan_coverage, swept_area  # noqa: E402
from mowbite_planner.geometry import polygons  # noqa: E402
from mowbite_planner.mapfile import area_job  # noqa: E402
from mowbite_planner.planner import (ANGLE_STRATEGIES, FILL_PATTERNS, PERIMETER_DIRECTIONS,  # noqa: E402
                                     PERIMETER_ORDERS)

# categorical colours (validated for colour blindness, all pairs), the line style tells them apart as well
STYLE = {
    'perimeter': dict(color='#2a78d6', lw=1.4, ls='-', label='perimeter loop'),
    'lane': dict(color='#1baf7a', lw=1.0, ls='-', label='lane'),
    'turn': dict(color='#eb6834', lw=1.2, ls='-', label='turn'),
    'turn_reverse': dict(color='#eb6834', lw=1.6, ls=(0, (2, 1.5)), label='turn, backing up'),
    'transit': dict(color='#4a3aa7', lw=1.4, ls=(0, (1, 1.5)), label='transit'),
}


def demo():
    """An L-shaped garden with a tree, a flower bed and a slanted edge."""
    boundary = [(0, 0), (22, 0), (24, 9), (13, 9), (13, 16), (0, 16)]
    tree = [(5 + 0.9 * math.cos(a * math.pi / 8), 6 + 0.9 * math.sin(a * math.pi / 8)) for a in range(16)]
    bed = [(15, 2.5), (19, 2.5), (19, 5), (15, 5)]
    return boundary, [tree, bed]


def from_json(path):
    with open(path) as f:
        data = json.load(f)
    return data['boundary'], data.get('obstacles', [])


def from_openmower_map(path, which):
    """An area of OpenMower's map.json by id or name (default the first), see mapfile.area_job."""
    with open(path) as f:
        data = json.load(f)
    try:
        return area_job(data, 0 if which is None else which)
    except PlanningError as e:
        sys.exit(str(e))


def draw_polygonal(ax, geom, **kw):
    for poly in polygons(geom):
        for ring in [poly.exterior, *poly.interiors]:
            xs, ys = ring.xy
            ax.plot(xs, ys, **kw)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument('--json', help='file with "boundary" and "obstacles"')
    src.add_argument('--map', help="OpenMower's map.json")
    ap.add_argument('--area', help='with --map: id or name of the mowing area (default: the first)')
    ap.add_argument('--width', type=float, default=0.22, help='mower_width in m')
    ap.add_argument('--overlap', type=float, default=0.15, help='overlap, 0.15 = 15%%')
    ap.add_argument('--passes', type=int, default=2, help='perimeter_passes')
    ap.add_argument('--radius', type=float, default=0.25, help='turn_radius in m')
    ap.add_argument('--no-reverse', action='store_true', help='no K-turns')
    ap.add_argument('--lane-order', default='snake', choices=['snake', 'skip'], help='lane_order')
    ap.add_argument('--offset', type=float, help='perimeter_offset in m (default: half the width)')
    ap.add_argument('--angle', type=float, help='lane angle in degrees (default: automatic)')
    ap.add_argument('--angle-strategy', '--strategy', default='longest_edge', choices=ANGLE_STRATEGIES)
    ap.add_argument('--angle-range', type=float, nargs=2, metavar=('MIN', 'MAX'),
                    help='keep the lane angle within MIN..MAX degrees')
    ap.add_argument('--pattern', default='lanes', choices=FILL_PATTERNS, help='fill_pattern')
    ap.add_argument('--perimeter-order', default='first', choices=PERIMETER_ORDERS)
    ap.add_argument('--direction', default='auto', choices=PERIMETER_DIRECTIONS, help='perimeter_direction')
    ap.add_argument('--turn-types', help='turns allowed, preferred first, e.g. bulb,k_turn,pivot')
    ap.add_argument('--end', type=float, nargs=2, metavar=('X', 'Y'), help='finish close to this point')
    ap.add_argument('--corner-radius', type=float, default=0.15, help='perimeter_corner_radius in m')
    ap.add_argument('--spacing', type=float, default=0.10, help='waypoint_spacing in m')
    ap.add_argument('--arrows', type=int, default=25, help='a heading arrow every n poses, 0 = none')
    ap.add_argument('--coverage', action='store_true', help='shade what the blade covers (slower)')
    ap.add_argument('--save', help='write the figure to this file instead of opening a window')
    args = ap.parse_args()

    if args.save:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    drivable, start, end = [], None, args.end
    if args.json:
        boundary, obstacles = from_json(args.json)
    elif args.map:
        job = from_openmower_map(args.map, args.area)
        boundary, obstacles, drivable = job.boundary, job.obstacles, job.drivable
        # from the dock and back to it
        start, end = job.dock, end or (job.dock[:2] if job.dock else None)
    else:
        boundary, obstacles = demo()

    lo, hi = [math.radians(a) for a in args.angle_range] if args.angle_range else (None, None)
    try:
        cfg = PlannerConfig(
            mower_width=args.width, overlap=args.overlap, perimeter_passes=args.passes,
            turn_radius=args.radius, allow_reverse=not args.no_reverse, lane_order=args.lane_order,
            perimeter_offset=args.offset, fill_pattern=args.pattern, perimeter_order=args.perimeter_order,
            perimeter_direction=args.direction, angle=None if args.angle is None else math.radians(args.angle),
            angle_strategy=args.angle_strategy, angle_min=lo, angle_max=hi,
            perimeter_corner_radius=args.corner_radius, waypoint_spacing=args.spacing)
        if args.turn_types:
            cfg = cfg.with_overrides({'turn_types': args.turn_types})
        plan = plan_coverage(boundary, obstacles, cfg, start, end, drivable)
    except PlanningError as e:
        sys.exit('planning failed: {}'.format(e))

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.set_aspect('equal')
    ax.set_facecolor('#fcfcfb')
    ax.grid(color='#e4e3df', lw=0.5)
    ax.set_axisbelow(True)

    if args.coverage:
        for poly in polygons(swept_area(plan, args.width).intersection(plan.free_space)):
            ax.fill(*poly.exterior.xy, color='#1baf7a', alpha=0.12, lw=0)
    for d in drivable:
        xs, ys = zip(*(list(d) + [d[0]]))
        # alpha only on the fill, the dashed edge stays visible
        ax.fill(xs, ys, facecolor='#52514e14', edgecolor='#52514e', lw=0.6, ls=(0, (2, 2)))
    for poly in polygons(plan.area):
        ax.fill(*poly.exterior.xy, color='#1baf7a', alpha=0.06, lw=0)
    draw_polygonal(ax, plan.area, color='#0b0b0b', lw=1.6)
    for o in obstacles:
        if len(o) >= 3:
            xs, ys = zip(*(list(o) + [o[0]]))
            ax.fill(xs, ys, facecolor='#e34948', alpha=0.25, edgecolor='#e34948', hatch='//', lw=1.0)
    draw_polygonal(ax, plan.lane_region, color='#52514e', lw=0.6, ls=(0, (4, 3)))

    for seg in plan.segments:
        key = 'turn_reverse' if seg.reverse else seg.kind
        style = {k: v for k, v in STYLE[key].items() if k != 'label'}
        pts = plan.poses[seg.start:seg.end + 1]
        ax.plot([p.x for p in pts], [p.y for p in pts], solid_capstyle='round', **style)

    if args.arrows > 0:
        sel = plan.poses[::args.arrows]
        ax.quiver([p.x for p in sel], [p.y for p in sel], [math.cos(p.yaw) for p in sel],
                  [math.sin(p.yaw) for p in sel], angles='xy', scale_units='xy', scale=1 / 0.35,
                  width=0.0018, color='#52514e', zorder=3)
    first, last = plan.poses[0], plan.poses[-1]
    ax.plot(first.x, first.y, 'o', ms=9, color='#2a78d6', mec='white', mew=1.5, zorder=4)
    ax.annotate('start', (first.x, first.y), xytext=(6, 6), textcoords='offset points', color='#0b0b0b')
    ax.plot(last.x, last.y, 's', ms=8, color='#0b0b0b', mec='white', mew=1.5, zorder=4)
    ax.annotate('end', (last.x, last.y), xytext=(6, 6), textcoords='offset points', color='#0b0b0b')
    if end is not None:
        ax.plot(end[0], end[1], 'x', ms=9, mew=2, color='#0b0b0b', zorder=4)

    handles = [Line2D([], [], **{k: v for k, v in s.items()}) for s in STYLE.values()]
    handles.append(Line2D([], [], color='#52514e', lw=0.6, ls=(0, (4, 3)), label='edge of the lane region'))
    ax.legend(handles=handles, loc='upper right', fontsize=8, framealpha=0.95)

    st = plan.stats
    turns = ', '.join('{} {}'.format(int(st['turns_' + k]), k.replace('_', '-'))
                      for k in ('u_turn', 'bulb', 'k_turn', 'detour', 'pivot', 'fallback') if st['turns_' + k])
    fill = 'concentric loops' if args.pattern == 'concentric' else 'lanes at ' + ' and '.join(
        '{:.1f}°'.format(math.degrees(a)) for a in plan.angles)
    ax.set_title('{}, spacing {:.3f} m · {:.0f} m path ({:.0f} m lanes, {:.0f} m perimeter, '
                 '{:.0f} m turns, {:.0f} m transit)\n{} lanes in {} cells · turns: {} · {} poses · planned in '
                 '{:.0f} ms'.format(fill, plan.lane_spacing, st['path_length'],
                                    st['lane_length'], st['perimeter_length'], st['turn_length'],
                                    st['transit_length'], int(st['lanes']), int(st['cells']), turns or 'none',
                                    int(st['poses']), 1000 * st['planning_time']),
                 fontsize=9, color='#0b0b0b')
    ax.set_xlabel('x [m] (map)')
    ax.set_ylabel('y [m] (map)')

    print('angle {} deg, spacing {:.3f} m'.format(
        ', '.join('{:.1f}'.format(math.degrees(a)) for a in plan.angles), plan.lane_spacing))
    for k, v in st.items():
        print('{:18} {}'.format(k, round(v, 3) if isinstance(v, float) else v))
    for w in plan.warnings:
        print('warning:', w)
    if args.coverage:
        covered = swept_area(plan, args.width).intersection(plan.free_space).area
        print('covered {:.2f} % of the mowable area'.format(100 * covered / plan.free_space.area))

    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=150)
        print('saved', args.save)
    else:
        plt.show()


if __name__ == '__main__':
    main()
