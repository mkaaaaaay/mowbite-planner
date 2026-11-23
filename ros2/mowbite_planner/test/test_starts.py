"""Tests of starts.py: a plan's sections where a robot that drives to them itself gets to them and away again (MowBite
OS), backing up out of a dead end (back_out)."""

import math
import os
import sys

import pytest
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner import PlannerConfig, PlanningError, plan_coverage  # noqa: E402
from mowbite_planner.config import LANE, PERIMETER, TRANSIT, CoveragePlan, SegmentInfo  # noqa: E402
from mowbite_planner.export import fingerprint, plan_pieces  # noqa: E402
from mowbite_planner.mathutil import Pose2D  # noqa: E402
from mowbite_planner.remote import refused  # noqa: E402
from mowbite_planner.report import rules  # noqa: E402
from mowbite_planner.starts import (ARRIVE_ASKS, BACK_OUT_MAX, START_UNREACHABLE, _Spent, backed_out,  # noqa: E402
                                    reached_starts)

# a hedge across the lawn: two sections, the path jumps from one to the other
LAWN = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
HEDGE = [(4.0, -1.0), (6.0, -1.0), (6.0, 7.0), (4.0, 7.0)]
CFG = PlannerConfig(mower_width=0.3, turn_radius=0.3)
_plans = {}


def plan():
    if 'split' not in _plans:
        _plans['split'] = plan_coverage(LAWN, [HEDGE], CFG)
    return _plans['split']


def sections(p):
    ends = list(p.section_starts[1:]) + [len(p.poses)]
    return [p.poses[a:b] for a, b in zip(p.section_starts, ends)]


def test_every_start_reached_leaves_the_plan():
    p = plan()
    asked = []
    got = reached_starts(p, lambda pose, came: asked.append((pose, came)) or True)
    assert got.poses == p.poses and got.segments == p.segments and got.warnings == p.warnings
    # each section's start (without the body nothing checked where they end)
    assert [pose for pose, _ in asked] == [p.poses[i] for i in p.section_starts]
    assert got.stats['starts'] == {'places': [], 'dropped': [], 'untried': [], 'ends': [], 'trapped': [],
                                   'unmowed_m2': 0.0, 'asks': 2}


def ends(p):
    """Each section's last pose."""
    return [max((s for s in p.segments if s.section == p.segments[k].section), key=lambda s: s.end).end
            for k in range(len(p.segments)) if k == 0 or p.segments[k].section != p.segments[k - 1].section]


def test_where_the_robot_comes_from():
    # the first section from wherever the robot is (None), each other one from where the section before it that's kept
    # ends: a section left out leaves the robot where it was
    if 'three' not in _plans:
        _plans['three'] = plan_coverage([(0.0, 0.0), (14.0, 0.0), (14.0, 6.0), (0.0, 6.0)],
                                        [[(4.0, -1.0), (5.0, -1.0), (5.0, 7.0), (4.0, 7.0)],
                                         [(9.0, -1.0), (10.0, -1.0), (10.0, 7.0), (9.0, 7.0)]], CFG)
    p = _plans['three']
    assert len(p.section_starts) == 3
    asked = []
    reached_starts(p, lambda pose, came: asked.append((pose, came)) or True)
    assert [came for _, came in asked] == [None, p.poses[ends(p)[0]], p.poses[ends(p)[1]]]
    gone = {(q.x, q.y, q.yaw) for q in sections(p)[1]}
    asked.clear()
    got = reached_starts(p, lambda pose, came: asked.append((pose, came)) or (pose.x, pose.y, pose.yaw) not in gone)
    assert len(got.section_starts) == 2 and len(got.stats['starts']['dropped']) == 1
    third = {(q.x, q.y, q.yaw) for q in sections(p)[2]}
    froms = [came for pose, came in asked if (pose.x, pose.y, pose.yaw) in third]
    assert froms and all(came is p.poses[ends(p)[0]] for came in froms)


def test_the_first_section_from_the_start():
    # arrives.start (MowBite OS: where it undocks to): the first section got to from there, a section left out leaves
    # the robot there; none got to from there: refused with START_UNREACHABLE first
    test_where_the_robot_comes_from()
    p = _plans['three']
    first = {(q.x, q.y, q.yaw) for q in sections(p)[0]}

    def asking(gets):
        asked = []

        def arrives(pose, came):
            asked.append((pose, came))
            return gets(pose, came)
        arrives.start = (12.5, 3.0, math.pi)
        return arrives, asked

    arrives, asked = asking(lambda pose, came: True)
    reached_starts(p, arrives)
    assert (asked[0][1].x, asked[0][1].y, asked[0][1].yaw) == (12.5, 3.0, math.pi)
    arrives, asked = asking(lambda pose, came: came.x != 12.5 or (pose.x, pose.y, pose.yaw) not in first)
    got = reached_starts(p, arrives)
    assert len(got.section_starts) == 2 and len(got.stats['starts']['dropped']) == 1
    second = {(q.x, q.y, q.yaw) for q in sections(p)[1]}
    assert any(came.x == 12.5 for pose, came in asked if (pose.x, pose.y, pose.yaw) in second)
    arrives, _ = asking(lambda pose, came: came.x != 12.5)
    with pytest.raises(PlanningError) as e:
        reached_starts(p, arrives)
    assert str(e.value).startswith(START_UNREACHABLE + 'no way from the start (12.50, 3.00) gets to any section')
    assert e.value.word == 'START_UNREACHABLE'  # (~/plan_coverage's code)
    # (the asks running out before one is: the same)
    arrives, asked = asking(lambda pose, came: came.x != 12.5)

    def spent(pose, came):
        if len(asked) >= 14:
            raise _Spent()
        return arrives(pose, came)
    spent.start = arrives.start
    with pytest.raises(PlanningError) as e:
        reached_starts(p, spent)
    assert str(e.value).startswith(START_UNREACHABLE) and str(e.value).endswith('ran out before the last ones)')
    assert e.value.word == 'START_UNREACHABLE'
    # without a start as ever
    with pytest.raises(PlanningError, match='^no way gets to any section') as e:
        reached_starts(p, lambda pose, came: False)
    assert e.value.word == ''


def test_a_section_starts_later_where_it_gets_to():
    p = plan()
    first = p.poses[p.section_starts[1]]
    # the second section's first metre can't be got to
    far = [k for k in range(p.section_starts[1], len(p.poses))
           if math.hypot(p.poses[k].x - first.x, p.poses[k].y - first.y) > 1.0][0]
    blocked = {(q.x, q.y, q.yaw) for q in p.poses[p.section_starts[1]:far]}
    got = reached_starts(p, lambda pose, came: (pose.x, pose.y, pose.yaw) not in blocked)
    assert len(got.section_starts) == 2
    later = got.poses[got.section_starts[1]]
    assert (later.x, later.y, later.yaw) not in blocked and math.hypot(later.x - first.x, later.y - first.y) > 1.0
    # the poses before it gone, the rest as planned, the segments along
    assert got.poses[got.section_starts[1]:] == p.poses[p.poses.index(later):]
    assert got.segments[0].start == 0 and got.segments[-1].end == len(got.poses) - 1
    assert all(a.end in (b.start, b.start - 1) for a, b in zip(got.segments, got.segments[1:]))
    places = got.stats['starts']['places']
    assert len(places) == 1 and places[0][:2] == [round(first.x, 2), round(first.y, 2)] and places[0][2] >= 1.0
    assert got.stats['starts']['unmowed_m2'] > 0.0
    assert any('start later' in w for w in got.warnings)


def test_a_section_it_gets_to_nowhere_is_left_out():
    p = plan()
    gone = {(q.x, q.y, q.yaw) for q in sections(p)[1]}
    got = reached_starts(p, lambda pose, came: (pose.x, pose.y, pose.yaw) not in gone)
    # no start it can't get to: the section is left out, not started as planned
    assert got.section_starts == [0] and got.poses == sections(p)[0]
    assert got.stats['starts']['dropped'] == [[round(sections(p)[1][0].x, 2), round(sections(p)[1][0].y, 2)]]
    assert any('left out' in w for w in got.warnings)
    with pytest.raises(PlanningError, match='no way gets to any section'):
        reached_starts(p, lambda pose, came: False)


def test_the_asks_are_counted():
    p = plan()
    asked = []
    with pytest.raises(PlanningError):
        reached_starts(p, lambda pose, came: bool(asked.append(pose)))
    # every section's first pose and up to ARRIVE_TRIES - 1 later ones, never more than ARRIVE_ASKS (a number: the same
    # plan on any computer; a section not asked about past that is left out too)
    assert 2 < len(asked) <= ARRIVE_ASKS


def test_plan_coverage_passes_arrives():
    seen = []
    got = plan_coverage(LAWN, [HEDGE], CFG, arrives=lambda pose, came: seen.append(pose) or True)
    assert got.poses == plan().poses and len(seen) == 2 and 'starts' in got.stats
    assert 'starts' not in plan().stats


# dead ends: a corridor 0.8 m wide off a square lawn, the body (0.4 wide, 0.6 long) turns on the spot in the square
# only; a lane in along the corridor to its end
BODY = dict(mower_width=0.3, robot_width=0.4, robot_front=0.45, robot_rear=0.15)


def corridor(sections=((1.0, 8.5),), back_out=False, extra=()):
    """A plan along y = 2 from x to x (each a section), the corridor from x 3.9 to 9 (and extra boxes)."""
    space = unary_union([box(0.0, 0.0, 4.0, 4.0), box(3.9, 1.6, 9.0, 2.4)] + [box(*b) for b in extra])
    poses, segments = [], []
    for k, (x0, x1) in enumerate(sections):
        n = int(round(abs(x1 - x0) / 0.1))
        yaw = 0.0 if x1 > x0 else math.pi
        first = len(poses)
        poses += [Pose2D(x0 + (x1 - x0) * i / n, 2.0, yaw) for i in range(n + 1)]
        segments.append(SegmentInfo(LANE, first, len(poses) - 1, False, '', k))
    return CoveragePlan(poses=poses, segments=segments, angle=0.0, lane_spacing=0.3, perimeter_loops=[], lanes=[],
                        stats={}, warnings=[], free_space=space, robot_space=space, transit_space=space,
                        body_space=space, config=PlannerConfig(back_out=back_out, **BODY),
                        section_starts=[s.start for s in segments])


def out_of_square(pose, came):
    """arrives of a robot that gets anywhere from the square, from the corridor nowhere (it doesn't turn there)."""
    return came is None or came.x < 3.5


def backs_up_right(plan):
    """Every step of the segments driven backwards: both poses face against it (FTC backs up there), the others along
    it; the plan's rules hold."""
    back = [False] * len(plan.poses)
    for s in plan.segments:
        back[s.start:s.end] = [s.reverse] * (s.end - s.start)
    for i, (a, b) in enumerate(zip(plan.poses, plan.poses[1:])):
        if i + 1 in plan.section_starts:
            continue
        d = math.hypot(b.x - a.x, b.y - a.y)
        along = [((b.x - a.x) * math.cos(p.yaw) + (b.y - a.y) * math.sin(p.yaw)) / d for p in (a, b)]
        assert all(f < 0 for f in along) if back[i] else along[0] > 0, (i, a, b)
    assert rules(plan)['hard'] == []


def test_a_section_ends_earlier_where_it_gets_away():
    # MowBite OS till M8: the lane's end in the corridor is a dead end, the section ends at the last pose before it the
    # robot gets away from (here: turning on the spot, the square's edge, or home), the rest left out
    p = corridor()
    asked = []
    got = reached_starts(p, lambda pose, came: asked.append(pose) or out_of_square(pose, came))
    end = got.poses[-1]
    # (away to the closest place the body turns whatever way, facing any way: in the square by the corridor)
    assert all(q.yaw is None and 3.0 < q.x < 4.0 and abs(q.y - 2.0) < 0.05 for q in asked[1:]) and len(asked) > 1
    assert got.poses == p.poses[:len(got.poses)] and 3.0 < end.x < 3.9
    assert got.stats['starts']['ends'] == [[8.5, 2.0, round(8.5 - end.x, 2)]]
    assert got.stats['starts']['unmowed_m2'] > 1.0 and 'back_out' not in got.stats
    assert any('end earlier' in w for w in got.warnings) and any('more unmowed' in w for w in got.warnings)
    assert got.stats['starts']['asks'] <= 1 + 12


def test_a_section_nothing_gets_away_from_is_left_out():
    # the second section in the corridor and out again only backing up: left out, the third (in the square) got to from
    # where the first ends
    p = corridor(((1.0, 3.0), (4.5, 8.5), (2.8, 1.5)))
    asked = []

    def arrives(pose, came):
        asked.append((pose, came))
        return came is None or came.x < 3.5 or pose.yaw is not None and pose.x > 4.0

    got = reached_starts(p, arrives)
    first, third = p.poses[:21], p.poses[p.section_starts[2]:]
    assert got.poses == first + third and len(got.section_starts) == 2
    assert got.stats['starts']['trapped'] == [[8.5, 2.0]] and got.stats['starts']['ends'] == []
    assert any(pose == third[0] and came == first[-1] for pose, came in asked)
    assert any('no way gets away from them' in w for w in got.warnings)
    with pytest.raises(PlanningError, match='and away from it again'):
        reached_starts(corridor(((4.5, 8.5),)), lambda pose, came: came is None)


def test_back_out_backs_up_to_where_it_gets_away():
    # with back_out (a robot that backs up where the plan does): the whole lane, then back along it to where it gets
    # away, each pose facing the way it did
    p = corridor(back_out=True)
    got = reached_starts(p, out_of_square, back_out=True)
    n = len(p.poses)
    k = len(got.poses) - n  # (poses backing up)
    assert got.poses[:n] == p.poses and got.poses[n:] == p.poses[n - 2:n - 2 - k:-1]
    assert 3.0 < got.poses[-1].x < 3.9 and got.stats['starts']['ends'] == []
    back = got.segments[-1]
    assert (back.start, back.end, back.reverse, back.maneuver, back.blade) == \
        (n - 1, len(got.poses) - 1, True, 'back_out', True)
    assert got.stats['back_out']['places'] == [[8.5, 2.0, round(8.5 - got.poses[-1].x, 2)]]
    assert got.stats['back_out']['corners'] == 0 and fingerprint(got) != fingerprint(p)
    assert any('back up out of a dead end' in w for w in got.warnings)
    backs_up_right(got)
    # (not further than BACK_OUT_MAX: it ends earlier then)
    far = corridor(((1.0, 4.0 + BACK_OUT_MAX + 1.0),), back_out=True, extra=[(3.9, 1.6, 45.0, 2.4)])
    got = reached_starts(far, out_of_square, back_out=True)
    assert not any(s.reverse for s in got.segments) and len(got.stats['starts']['ends']) == 1
    # (a section after it that's left out: away again from where it backed up to, once)
    two = corridor(((1.0, 8.5), (4.5, 8.0)), back_out=True)
    got = reached_starts(two, lambda pose, came: came is None or came.x < 3.5, back_out=True)
    assert len(got.stats['back_out']['places']) == 1 and got.stats['starts']['trapped'] == [[8.0, 2.0]]
    assert got.poses[-1].x < 3.9 and len(got.section_starts) == 1


def test_backed_out_where_the_turn_on_the_spot_doesnt_fit():
    # OpenMower (no arrives): its FTC turns on the spot where a section ends; it doesn't fit in the corridor, the plan
    # backs up to the first pose it does
    p = corridor(back_out=True)
    got = backed_out(p)
    n = len(p.poses)
    k = len(got.poses) - n
    assert got.poses[:n] == p.poses and got.poses[n:] == p.poses[n - 2:n - 2 - k:-1]
    assert 3.0 < got.poses[-1].x < 3.9 and got.stats['back_out']['places'][0][:2] == [8.5, 2.0]
    backs_up_right(got)
    # where it ends in the square nothing changes; where no pose behind it fits it stays as it was
    p = corridor(((1.0, 2.0),), back_out=True)
    assert backed_out(p).poses == p.poses and backed_out(p).stats['back_out'] == {'places': [], 'stuck': []}
    p = corridor(((4.5, 8.5),), back_out=True)
    assert backed_out(p).poses == p.poses and backed_out(p).stats['back_out']['stuck'] == [[8.5, 2.0]]


def test_backing_up_round_a_sharp_corner():
    # the corridor turns 120 degrees into an arm (the body turns that far there, not all the way round): backing up out
    # of the arm the pose at the corner faces between both ways, one 2 mm on along the next leg against it (FTC backs
    # up from a pose where it and the next one both face against the way)
    a = math.radians(120.0)
    ux, uy, nx, ny = math.cos(a), math.sin(a), math.sin(a), -math.cos(a)
    arm = Polygon([(6.0 + 0.4 * nx, 2.0 + 0.4 * ny), (6.0 + 3.6 * ux + 0.4 * nx, 2.0 + 3.6 * uy + 0.4 * ny),
                   (6.0 + 3.6 * ux - 0.4 * nx, 2.0 + 3.6 * uy - 0.4 * ny), (6.0 - 0.4 * nx, 2.0 - 0.4 * ny)])
    p = corridor(((1.0, 6.0),), back_out=True, extra=[(5.2, 1.6, 6.8, 2.8)])
    space = p.body_space.union(arm)
    poses = p.poses[:-1] + [Pose2D(6.0, 2.0, a)] + [Pose2D(6.0 + 0.1 * i * ux, 2.0 + 0.1 * i * uy, a)
                                                    for i in range(1, 31)]
    turned = CoveragePlan(**dict(vars(p), poses=poses, segments=[
        SegmentInfo(LANE, 0, len(p.poses) - 1, False, '', 0), SegmentInfo(TRANSIT, len(p.poses) - 1, len(poses) - 1,
                                                                          False, '', 0)],
        free_space=space, robot_space=space, transit_space=space, body_space=space))
    assert rules(turned)['hard'] == []
    got = reached_starts(turned, out_of_square, back_out=True)
    assert got.stats['back_out']['corners'] == 1 and got.poses[-1].x < 3.9
    at = next(i for i, q in enumerate(got.poses) if i >= len(poses) and (q.x, q.y) == (6.0, 2.0))
    assert math.degrees(got.poses[at].yaw) == pytest.approx(60.0) and got.poses[at + 1].yaw == pytest.approx(0.0)
    backs_up_right(got)


def test_backing_up_round_a_slight_bend():
    # round a bend of 20 degrees each pose backing up faces against its step back (the heading it had going forwards
    # there): the plan's form holds as the house computer's plans are checked (remote.refused), FTC backs up all along
    a = math.radians(20.0)
    ux, uy, nx, ny = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
    arm = Polygon([(6.0 - 0.4 * nx, 2.0 - 0.4 * ny), (6.0 + 3.0 * ux - 0.4 * nx, 2.0 + 3.0 * uy - 0.4 * ny),
                   (6.0 + 3.0 * ux + 0.4 * nx, 2.0 + 3.0 * uy + 0.4 * ny), (6.0 + 0.4 * nx, 2.0 + 0.4 * ny)])
    joint = Polygon([(6.0 - 0.4 * nx, 2.0 - 0.4 * ny), (6.0, 1.6), (6.0, 2.4), (6.0 + 0.4 * nx, 2.0 + 0.4 * ny)])
    space = unary_union([box(0.0, 0.0, 4.0, 4.0), box(3.9, 1.6, 6.0, 2.4), arm, joint.convex_hull])
    p = corridor(((1.0, 6.0),), back_out=True)
    poses = p.poses[:-1] + [Pose2D(6.0, 2.0, a)] + [Pose2D(6.0 + 0.1 * i * ux, 2.0 + 0.1 * i * uy, a)
                                                    for i in range(1, 26)]
    bent = CoveragePlan(**dict(vars(p), poses=poses, segments=[SegmentInfo(LANE, 0, len(poses) - 1, False, '', 0)],
                               free_space=space, robot_space=space, transit_space=space, body_space=space))
    assert refused(bent) == ''
    for got in (reached_starts(bent, out_of_square, back_out=True), backed_out(bent)):
        assert got.stats['back_out']['places'] and refused(got) == ''
        backs_up_right(got)


def test_a_section_going_on_where_one_ends():
    # a section starting on the last pose of the one before (left out: nothing of it gets away): that pose stays the
    # one before's, backing up from there once, no segment of the one left out
    p = corridor(((1.0, 6.0),), back_out=True)
    k = len(p.poses) - 1
    poses = p.poses + [Pose2D(6.0 + 0.1 * i, 2.0, 0.0) for i in range(1, 26)]
    two = CoveragePlan(**dict(vars(p), poses=poses, segments=[SegmentInfo(LANE, 0, k, False, '', 0),
                                                              SegmentInfo(LANE, k, len(poses) - 1, False, '', 1)],
                              section_starts=[0, k]))
    for back_out in (False, True):
        got = reached_starts(two, out_of_square, back_out=back_out)
        assert got.stats['starts']['trapped'] == [[8.5, 2.0]] and got.poses[:20] == p.poses[:20]
        assert len(got.section_starts) == 1 and all(s.start < s.end for s in got.segments)
        assert len([s for s in got.segments if s.reverse]) == (1 if back_out else 0)
        assert got.poses[-1].x < 3.9
        backs_up_right(got)


def test_back_out_ends_earlier_where_it_backs_up_to_nothing():
    # a section from just past where the body turns having backed up to it: backing up gets nowhere, it ends earlier
    # as without back_out (not left out)
    for back_out in (False, True):
        p = corridor(back_out=back_out)
        poses = [Pose2D(3.54 + 0.01 * i, 2.0, 0.0) for i in range(486)]
        fine = CoveragePlan(**dict(vars(p), poses=poses, segments=[SegmentInfo(LANE, 0, 485, False, '', 0)]))
        got = reached_starts(fine, lambda pose, came: came is None, back_out=back_out)
        assert len(got.stats['starts']['ends']) == 1 and got.stats['starts']['trapped'] == []
        assert not any(s.reverse for s in got.segments)


def test_ending_earlier_on_a_segments_end():
    # cut back to where a segment ends: no segment of one pose left after it
    p = corridor()
    k = next(i for i, q in enumerate(p.poses) if q.x >= 3.6 - 1e-9)
    split = CoveragePlan(**dict(vars(p), segments=[SegmentInfo(LANE, 0, k, False, '', 0),
                                                   SegmentInfo(TRANSIT, k, len(p.poses) - 1, False, '', 0)]))
    got = reached_starts(split, out_of_square)
    assert got.stats['starts']['ends'] and all(s.start < s.end for s in got.segments)
    assert refused(got) == ''


def test_away_to_where_it_turns_in_its_own_part():
    # the closest place it turns lies across a hedge (another part of the room): asked about one in its own part
    p = corridor(extra=[(4.5, 2.6, 9.0, 6.0)])
    hedge = box(4.5, 2.4, 9.0, 2.6)
    parted = CoveragePlan(**dict(vars(p), transit_space=p.transit_space.difference(hedge)))
    asked = []
    reached_starts(parted, lambda pose, came: asked.append(pose) or out_of_square(pose, came))
    assert asked[1:] and all(q.x < 4.0 for q in asked[1:])


def test_back_out_pieces_go_with_what_they_back_out_of():
    # OpenMower's paths: backing up from a lane onto the loop before it stays the lane's path, however short
    p = corridor(back_out=True)
    k = next(i for i, q in enumerate(p.poses) if q.x >= 5.0)
    loop = CoveragePlan(**dict(vars(p), segments=[SegmentInfo(PERIMETER, 0, k, False, '', 0),
                                                  SegmentInfo(LANE, k, len(p.poses) - 1, False, '', 0)]))
    got = reached_starts(loop, out_of_square, back_out=True)
    assert [s.kind for s in got.segments if s.maneuver == 'back_out'] == [LANE, PERIMETER]
    assert [piece[0] for piece in plan_pieces(got)] == [True, False] and plan_pieces(got)[-1][2] == len(got.poses) - 1


def test_reverse_only_backing_out():
    # the plan's rules: driving backwards in a plan forwards only where back_out backs up out of a dead end
    got = reached_starts(corridor(back_out=True), out_of_square, back_out=True)
    assert rules(got)['hard'] == []
    off = CoveragePlan(**dict(vars(got), config=got.config.with_overrides({'back_out': False})))
    assert rules(off)['hard'] == ['drives backwards']


def test_back_out_in_plan_coverage():
    # without the body set it's ignored (said so), with it the body is checked (body_fit forced as for slope and
    # reverse, said where it was off)
    cfg = PlannerConfig(mower_width=0.3, turn_radius=0.3, back_out=True)
    got = plan_coverage(LAWN, [HEDGE], cfg)
    assert got.poses == plan().poses and got.warnings[-1].startswith('back_out ignored: no body set')
    small = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.5), (0.0, 2.5)]
    got = plan_coverage(small, [], PlannerConfig(back_out=True, body_fit=False, **BODY))
    assert got.config.body_fit and got.warnings[-1].startswith('back_out: body_fit on') and 'back_out' in got.stats


def budget(n, answer):
    """arrives of a robot that answers like answer for the first n ways, then plans no more (_Spent, like MowBite OS'
    ARRIVE_WAYS)."""
    asked = []

    def arrives(pose, came):
        if len(asked) >= n:
            raise _Spent()
        asked.append((pose, came))
        return answer(pose, came)
    return arrives


def test_asks_run_out_where_a_section_ends():
    # the ways away from the lane's end in the corridor not asked any more: it ends at the last pose the body turns on
    # the spot at (as where they're asked and none gets away), told as untried, not as a dead end
    p = corridor()
    got = reached_starts(p, budget(1, out_of_square))
    end = got.poses[-1]
    assert got.poses == p.poses[:len(got.poses)] and 1.0 < end.x < 3.9
    starts = got.stats['starts']
    assert len(starts['ends']) == 1 and starts['ends'][0][:2] == [8.5, 2.0]
    assert starts['trapped'] == [] and starts['untried'] == [] and starts['unmowed_m2'] > 1.0
    assert any('end earlier where the ways away from their end were not tried past' in w for w in got.warnings)
    assert not any('no way gets away' in w for w in got.warnings)


def test_asks_run_out_in_a_dead_end():
    # the second section in the corridor got to, the ways away from it not asked any more: left out as untried (its
    # last pose), not trapped; the third not asked about either
    p = corridor(((1.0, 3.0), (4.5, 8.5), (2.8, 1.5)))
    got = reached_starts(p, budget(2, lambda pose, came: came is None or came.x < 3.5 or
                                   pose.yaw is not None and pose.x > 4.0))
    starts = got.stats['starts']
    assert got.poses == p.poses[:21] and starts['trapped'] == [] and starts['ends'] == []
    assert starts['untried'] == [[8.5, 2.0], [2.8, 2.0]]
    assert any(w.startswith('1 sections left out: the ways away from them not tried past') for w in got.warnings)
    assert any(w.startswith('1 sections left out: the ways to them not tried past') for w in got.warnings)
    assert not any('no way gets away' in w for w in got.warnings)
    # a start got to, nothing left: not START_UNREACHABLE (it got there), the asks running out said
    one = corridor(((4.5, 8.5),))
    arrives = budget(1, lambda pose, came: True)
    arrives.start = (1.0, 2.0, 0.0)
    with pytest.raises(PlanningError, match='^no way gets to any section .* and away from it again .*ran out') as e:
        reached_starts(one, arrives)
    assert e.value.word == ''
