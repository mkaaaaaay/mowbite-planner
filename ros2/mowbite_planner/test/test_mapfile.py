"""Tests of reading OpenMower's map.json and parameters (mapfile.py), without ROS."""

import math
import os
import struct
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), HERE]

from mowbite_planner import PlannerConfig, PlanningError  # noqa: E402
from mowbite_planner.mapfile import (MAP_FIELDS, area_job, area_settings, bodies_for, body_hints,  # noqa: E402
                                     body_known, body_warnings, drawn, find_area, margins_for, match_area, mow_areas, openmower_angle,
                                     openmower_settings, recorded_body, settings_for)
from stubplan import stub_plan_coverage  # noqa: E402


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def area(id_, kind, points, **props):
    return {'id': id_, 'properties': {'name': id_.title(), 'type': kind, **props},
            'outline': [{'x': x, 'y': y} for x, y in points]}


MAP = {
    'areas': [
        area('path', 'nav', rect(-4, 0, 0, 2)),
        area('front', 'mow', rect(0, 0, 10, 6), angle=0.5, outline_count=1),
        area('bed', 'obstacle', rect(4, 2, 5, 3)),
        area('old_bed', 'obstacle', rect(6, 2, 7, 3), active=False),
        area('pond', 'mow', rect(7, 4, 8, 5), mowable=False, mow_around=True),
        area('field', 'mow', rect(-1, -1, 11, 7), mowable=False, mow_around=True),  # around front, not in it
        area('back', 'mow', rect(0, 8, 6, 12)),
        area('shed', 'nav', rect(6, 8, 8, 10), active=False),
        area('line', 'obstacle', [(1, 1), (2, 2)]),  # no area
    ],
    'docking_stations': [{'id': 'broken'}, {'id': 'dock', 'position': {'x': -3.0, 'y': 1.0}, 'heading': math.pi}],
}


# areas -----------------------------------------------------------------------------------------------------


def test_find_area():
    assert [a['id'] for a in mow_areas(MAP)] == ['front', 'pond', 'field', 'back']
    assert find_area(MAP, 'front')['id'] == 'front'  # by id
    assert find_area(MAP, 'Back')['id'] == 'back'  # by name
    assert find_area(MAP, 0)['id'] == 'front'  # by index among the mowing areas
    assert find_area(MAP, 3)['id'] == 'back'
    for which in ('path', 'bed', 'nowhere', 4, -1, True):
        with pytest.raises(PlanningError, match='Front, Pond, Field, Back'):
            find_area(MAP, which)
    with pytest.raises(PlanningError, match='none'):
        find_area({}, 0)


def test_area_job():
    job = area_job(MAP, 'front')
    assert (job.area_id, job.name) == ('front', 'Front')
    assert job.boundary == rect(0, 0, 10, 6)
    # the active obstacle and the non-mowable area to mow around inside it, not the inactive one, not one
    # without an area, not the one around the whole area (that one isn't drivable either)
    assert job.obstacles == [rect(4, 2, 5, 3), rect(7, 4, 8, 5)]
    assert job.mow_around == [rect(7, 4, 8, 5)]  # OpenMower sends its planner the obstacles, not this one
    # the other mowing areas may be crossed, not the active nav area: OpenMower mows a path from start to end
    assert job.drivable == [rect(0, 8, 6, 12)]
    assert job.nav == [rect(-4, 0, 0, 2)]
    # the body may reach over the edge into the active nav area (not the inactive one)
    assert job.navigation == [rect(-4, 0, 0, 2)]
    # nothing goes over the inactive nav area on the way, the inactive obstacle is gone
    assert job.keep_out == [rect(6, 8, 8, 10)]
    assert job.dock == (-3.0, 1.0, math.pi)  # the first one with a position
    assert job.overrides == {'angle': 0.5, 'perimeter_passes': 1} and job.planner == {}
    # the one without an area left out, told (the plan's warnings get it)
    assert job.warnings == ['1 obstacles of the map left out, no area (fewer than 3 points, or all on one line), the '
                            'first at (1.00, 1.00)']
    # a mowing area not to mow is like a navigation area, unless it's one to mow around: the body may reach over into
    # it, the blade keeps off it (unmowed), nothing goes over it on the way
    meadow = area('meadow', 'mow', rect(0, 8, 6, 12), mowable=False)
    job = area_job({'areas': [MAP['areas'][1], meadow]}, 'front')
    assert job.nav == [rect(0, 8, 6, 12)] and not job.drivable and not job.obstacles and not job.mow_around
    assert job.unmowed == [rect(0, 8, 6, 12)] and job.keep_out == [] and job.navigation == []
    # an inactive mowing area is no way either
    lawn = area('lawn', 'mow', rect(0, 8, 6, 12), active=False)
    job = area_job({'areas': [MAP['areas'][1], lawn]}, 'front')
    assert not job.drivable and job.keep_out == [rect(0, 8, 6, 12)]


def test_obstacles_without_an_area_told():
    """An active obstacle in the area with fewer than 3 points is left out and told; not one elsewhere, an inactive
    one or one unreadable. One with 3 points on one line goes to the planner, which tells it."""
    front = MAP['areas'][1]
    for others, told in (([area('dot', 'obstacle', [(3, 4)])],
                          '1 obstacles of the map left out, no area (fewer than 3 points, or all on one line), the '
                          'first at (3.00, 4.00)'),
                         ([area('a', 'obstacle', [(3, 4), (5, 4)]), area('b', 'obstacle', [(2, 2)])],
                          '2 obstacles of the map left out, no area (fewer than 3 points, or all on one line), the '
                          'first at (3.00, 4.00)'),
                         ([area('far', 'obstacle', [(30, 4), (31, 4)])], None),
                         ([area('gone', 'obstacle', [(3, 4), (5, 4)], active=False)], None),
                         ([area('none', 'obstacle', [])], None),
                         ([{'id': 'broken', 'properties': {'type': 'obstacle'}, 'outline': 5}], None)):
        job = area_job({'areas': [front] + others}, 'front')
        assert job.obstacles == [] and job.warnings == ([told] if told else []), others
    line = [(1, 1), (2, 2), (3, 3)]
    job = area_job({'areas': [front, area('line', 'obstacle', line)]}, 'front')
    assert job.obstacles == [line] and job.warnings == []


def test_nested_areas():
    """A mowing area in another (most of it) is nested in it, not one next to it with an edge in common, not the
    other way round, not an inactive one or one not to mow."""
    data = {'areas': [area('lawn', 'mow', rect(0, 0, 10, 8)), area('zone', 'mow', rect(2, 2, 5, 5)),
                      area('edge', 'mow', rect(9.5, 3, 12, 6)),  # mostly outside
                      area('next', 'mow', rect(10, 0, 14, 8)),  # an edge in common
                      area('off', 'mow', rect(6, 1, 8, 3), active=False),
                      area('meadow', 'mow', rect(6, 5, 8, 7), mowable=False)]}
    assert area_job(data, 'lawn').nested == [rect(2, 2, 5, 5)]
    assert area_job(data, 'zone').nested == [] and area_job(data, 'next').nested == []
    assert rect(2, 2, 5, 5) in area_job(data, 'lawn').drivable  # still a way through


def test_match_area():
    # the index among the mowing areas, for area_job: ids may be missing or twice
    a, b = area('x', 'mow', rect(0, 0, 10, 6)), area('x', 'mow', rect(20, 0, 30, 6))
    no_id = {'properties': {'type': 'mow'}, 'outline': a['outline']}
    data = {'areas': [area('path', 'nav', rect(10, 0, 20, 2)), a, b, no_id]}
    # as OpenMower sends it: float32, maybe closed
    sent = [(x + 1e-6, y - 1e-6) for x, y in rect(20, 0, 30, 6)] + [(20.0, 0.0)]
    assert match_area(data, sent) == 1 and area_job(data, 1).boundary == rect(20, 0, 30, 6)
    assert match_area(data, rect(0, 0, 10, 6)) == 0  # the first with it
    assert match_area(data, rect(0, 0, 10, 7)) is None
    assert match_area(data, rect(10, 0, 20, 2)) is None  # not a mowing area
    # the first that isn't set to inactive or not mowable, else the first
    off, unmowed = area('x', 'mow', rect(0, 0, 10, 6), active=False), area('x', 'mow', rect(0, 0, 10, 6), mowable=False)
    assert match_area({'areas': [off, unmowed, a, b]}, rect(0, 0, 10, 6)) == 2
    assert match_area({'areas': [off, unmowed, b]}, rect(0, 0, 10, 6)) == 0
    assert match_area({'areas': [b, unmowed, off]}, rect(0, 0, 10, 6)) == 1


def test_margins_for():
    """An obstacle's or area's own margin, found by its outline as OpenMower sends it (float32, maybe closed). None for
    an outline the map doesn't have, and where it's no number 0..1 (10 meant as cm: the default rather than no plan)."""
    data = {'areas': [area('front', 'mow', rect(0, 0, 10, 6), planner={'edge_margin': 0.2, 'obstacle_margin': 0,
                                                                       'spin_margin': 0.08}),
                      area('bed', 'obstacle', rect(4, 2, 5, 3), margin=0.3),
                      area('tree', 'obstacle', rect(6, 2, 7, 3)),
                      area('pond', 'mow', rect(1, 1, 2, 2), mowable=False, margin=0),
                      area('odd', 'obstacle', rect(8, 2, 9, 3), margin=-0.1),
                      area('far', 'obstacle', rect(8, 4, 9, 5), margin='far'),
                      area('cm', 'obstacle', rect(8, 6, 9, 7), margin=10)]}
    sent = [struct.unpack('<2f', struct.pack('<2f', x + 1e-7, y)) for x, y in rect(4, 2, 5, 3)]
    outlines = [sent + sent[:1], rect(6, 2, 7, 3), rect(1, 1, 2, 2), rect(8, 2, 9, 3), rect(8, 4, 9, 5),
                rect(4, 2, 5, 4), rect(8, 6, 9, 7)]
    assert margins_for(data, outlines) == [0.3, None, 0.0, None, None, None, None]
    assert margins_for(None, outlines[:1]) == [None] and margins_for(data, []) == []
    # the area's own in its planner object
    cfg = settings_for(PlannerConfig(), None, area_job(data, 'front'))
    assert (cfg.edge_margin, cfg.obstacle_margin, cfg.spin_margin) == (0.2, 0.0, 0.08)


RECORDED = {'robot_width': 0.4, 'robot_front': 0.43, 'robot_rear': 0.14, 'robot_outline': [], 'by': 'os-1'}


def test_recorded_body():
    """An area's recorded_body as the body's corners (robot_outline beats the rectangle, by doesn't count), None
    without one, ValueError where it's no body."""
    assert recorded_body(area('a', 'mow', rect(0, 0, 1, 1))) is None
    assert recorded_body(area('a', 'mow', rect(0, 0, 1, 1), recorded_body=RECORDED)) == \
        PlannerConfig(robot_width=0.4, robot_front=0.43, robot_rear=0.14).body_outline
    kite = [[0.5, 0], [0, 0.2], [-0.1, 0], [0, -0.2]]
    assert recorded_body(area('a', 'mow', rect(0, 0, 1, 1), recorded_body=dict(RECORDED, robot_outline=kite))) == \
        ((0.5, 0.0), (0.0, 0.2), (-0.1, 0.0), (0.0, -0.2))
    for bad in ('nx60i', [0.4], {'robot_width': 0.4}, dict(RECORDED, robot_front='x'), dict(RECORDED, robot_width=-1),
                dict(RECORDED, robot_outline=[[1, 1], [2, 2]]), dict(RECORDED, robot_outline=5),
                dict(RECORDED, robot_outline=[[0.5, 0], [0, 'a'], [-0.1, 0]])):
        with pytest.raises(ValueError, match='recorded_body'):
            recorded_body(area('a', 'mow', rect(0, 0, 1, 1), recorded_body=bad))


def test_bodies_for_and_body_hints():
    """bodies_for: of the outlines (float32, closed) the ones of an area with a recorded_body, its corners; none
    unreadable. body_hints: of the areas with one of the outlines, larger where the planning body sticks out of the
    recorded one by more than 2 cm, unreadable, missing only where asked; none without a planning body."""
    narrow = dict(RECORDED, robot_width=0.3)
    data = {'areas': [area('front', 'mow', rect(0, 0, 10, 6), recorded_body=narrow),
                      area('bed', 'obstacle', rect(4, 2, 5, 3), recorded_body=RECORDED),
                      area('tree', 'obstacle', rect(6, 2, 7, 3)),
                      area('pond', 'mow', rect(1, 1, 2, 2), recorded_body={'robot_width': 'wide'}),
                      area('far', 'obstacle', rect(20, 2, 21, 3), recorded_body=narrow)]}
    sent = [struct.unpack('<2f', struct.pack('<2f', x + 1e-7, y)) for x, y in rect(0, 0, 10, 6)]
    outlines = [sent + sent[:1], rect(4, 2, 5, 3), rect(6, 2, 7, 3), rect(1, 1, 2, 2)]
    got = bodies_for(data, outlines)
    assert [o for o, _ in got] == outlines[:2]
    assert got[1][1] == recorded_body(data['areas'][1]) and bodies_for(None, outlines) == ()
    body = PlannerConfig(robot_width=0.4, robot_front=0.43, robot_rear=0.14).body_outline
    assert body_hints(data, outlines, body) == {'front': 'larger', 'pond': 'unreadable'}
    assert body_hints(data, outlines, body, missing=True) == {'front': 'larger', 'tree': 'missing',
                                                              'pond': 'unreadable'}
    near = PlannerConfig(robot_width=0.43, robot_front=0.44, robot_rear=0.15).body_outline
    smaller = PlannerConfig(robot_width=0.25, robot_front=0.3, robot_rear=0.1).body_outline
    assert body_hints(data, outlines[1:2], near) == body_hints(data, outlines[:1], smaller) == {}
    assert body_hints(data, outlines, None, missing=True) == {}
    assert body_warnings({'front': 'larger', 'bed': 'larger', 'tree': 'missing', 'pond': 'unreadable'}) == [
        'areas bed, front: recorded with a smaller body than this one, record again for the exact edge',
        'area pond: recorded_body unreadable, planned with this body']


def test_drawn_rings():
    """drawn: an area drawn (drawn true) is hard, centre only with drawn_edge centre (anything else hard), not drawn
    without drawn true whatever drawn_edge says. bodies_for has it as drawn whatever recorded_body it has, body_known
    not, body_hints none for it (not missing either)."""
    lawn = area('lawn', 'mow', rect(0, 0, 10, 6), drawn=True, drawn_edge='centre')
    data = {'areas': [lawn, area('bed', 'obstacle', rect(4, 2, 5, 3), drawn=True, recorded_body=RECORDED),
                      area('pond', 'obstacle', rect(1, 1, 2, 2), drawn=True, drawn_edge='wavy'),
                      area('tree', 'obstacle', rect(6, 2, 7, 3), drawn='yes', drawn_edge='centre'),
                      area('rose', 'obstacle', rect(7.5, 2, 8, 3), drawn_edge='hard')]}
    assert [drawn(a) for a in data['areas']] == ['centre', 'hard', 'hard', None, None]
    outlines = [rect(0, 0, 10, 6), rect(4, 2, 5, 3), rect(1, 1, 2, 2), rect(6, 2, 7, 3), rect(7.5, 2, 8, 3)]
    got = bodies_for(data, outlines)
    assert got == ((outlines[0], 'centre'), (outlines[1], 'hard'), (outlines[2], 'hard'))
    assert not body_known(got, outlines[1]) and body_known(((outlines[1], ((0.1, 0.0),)),), outlines[1])
    body = PlannerConfig(robot_width=0.6, robot_front=0.6, robot_rear=0.2).body_outline
    assert body_hints(data, outlines, body, missing=True) == {'tree': 'missing', 'rose': 'missing'}
    assert {'recorded_body', 'drawn', 'drawn_edge'} <= set(MAP_FIELDS)


def test_area_job_without_dock_or_properties():
    data = {'areas': [{'id': 7, 'outline': [[0, 0], [5, 0], [5, 5]], 'properties': {'type': 'mow'}}],
            'docking_stations': [{'position': {'x': 1, 'y': 2}}]}
    job = area_job(data, 0)
    assert (job.area_id, job.name) == ('7', '7')
    assert job.boundary == [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]
    assert job.dock == (1.0, 2.0, 0.0)
    assert not job.obstacles and not job.drivable and not job.nav and not job.overrides and not job.planner
    assert area_job({'areas': data['areas']}, 0).dock is None


def test_area_job_with_properties_of_a_request():
    # like OpenMower's mowing.plan: they replace the area's, the planner object stays
    data = {'areas': [area('a', 'mow', rect(0, 0, 10, 6), angle=0.5, outline_count=1, planner={'turn_radius': 0.3})]}
    job = area_job(data, 'a', {'angle': 0.2, 'angle_min': 0.1, 'angle_max': 0.4})
    assert job.overrides == {'angle': 0.2, 'perimeter_passes': 1, 'angle_min': 0.1, 'angle_max': 0.4,
                             'turn_radius': 0.3}
    assert job.planner == {'angle_min': 0.1, 'angle_max': 0.4, 'turn_radius': 0.3}
    assert data['areas'][0]['properties']['angle'] == 0.5  # the map stays as it is
    # the rest as without them: the area doesn't count among the other lawns
    for which in ('front', 'back'):
        plain, asked = area_job(MAP, which), area_job(MAP, which, {'angle': 0.2})
        assert asked.drivable == plain.drivable and asked.obstacles == plain.obstacles and asked.nav == plain.nav
    with pytest.raises(PlanningError, match="area 'a': .*perimeter_passes"):
        area_job(data, 'a', {'outline_count': 300})


def test_broken_maps():
    """A broken map is a PlanningError at worst: other areas that are broken are left out, a broken area to
    plan is an error that names it."""
    good = area('front', 'mow', rect(0, 0, 10, 6))
    p = {'x': 1, 'y': 1}
    for outline in (None, 5, 'x', {'x': 1}, [p, p], [p, {'x': 1}, p], [p, {'x': '2', 'y': 1}, p],
                    [p, {'x': None, 'y': 1}, p], [[0, 0], [1, math.nan], [1, 1]], [[0, 0], [math.inf, 1], [1, 1]],
                    [[0, 0], [1], [1, 1]], [[0, 0], [True, 1], [1, 1]], [[0, 0], 'ab', [1, 1]],
                    [[0, 0], [10 ** 400, 1], [1, 1]]):
        for kind in ('obstacle', 'nav', 'mow'):
            job = area_job({'areas': [good, {'id': 'broken', 'properties': {'type': kind}, 'outline': outline}]}, 0)
            assert not job.obstacles and not job.drivable and not job.nav
        broken = {'id': 'broken', 'properties': {'type': 'mow'}, 'outline': outline}
        assert match_area({'areas': [broken, good]}, rect(0, 0, 10, 6)) == 1
        with pytest.raises(PlanningError, match="area 'broken': .*(outline|point)"):
            area_job({'areas': [good, broken]}, 'broken')
    # what isn't a map, areas or properties: no mowing area there
    for data in (None, [good], 'map', {'areas': 5}, {'areas': {'front': good}},
                 {'areas': [dict(good, properties=['mow'])]}):
        assert match_area(data, rect(0, 0, 10, 6)) is None
        with pytest.raises(PlanningError, match='no mowing area'):
            area_job(data, 0)
    data = {'areas': [None, 'x', 5, [good], {'properties': None}, good]}
    assert [a['id'] for a in mow_areas(data)] == ['front'] and area_job(data, 'front').boundary == rect(0, 0, 10, 6)
    # an outline OpenMower sends that isn't one
    for outline in (None, [(0, 0), (math.nan, 0), (1, 1)], [(0, 0), 'x', (1, 1)]):
        assert match_area(MAP, outline) is None
    # docking stations that aren't one
    for docks in (5, 'x', {'position': {'x': 1, 'y': 2}},
                  [None, 'x', {'position': [1, 2]}, {'position': 'x'}, {'position': {'x': '1', 'y': 2}},
                   {'position': {'x': 1, 'y': math.nan}}, {'position': {'x': 1, 'y': 2}, 'heading': 'north'}]):
        assert area_job({'areas': [good], 'docking_stations': docks}, 0).dock is None
    # settings of the area that aren't any
    for props, what in (({'planner': {'turn_types': 5}}, 'turn_types'),
                        ({'planner': {'perimeter_passes': 'x'}}, 'perimeter_passes'),
                        ({'planner': {'mower_width': -1}}, 'mower_width'),
                        ({'planner': {'turn_on_spot': 'no'}}, 'turn_on_spot'),
                        ({'planner': {'colour': 'green'}}, 'colour'), ({'planner': 'concentric'}, 'planner'),
                        ({'planner': [1]}, 'planner'), ({'outline_count': 300}, 'perimeter_passes')):
        with pytest.raises(PlanningError, match=f"area 'front': .*{what}"):
            area_job({'areas': [area('front', 'mow', rect(0, 0, 10, 6), **props)]}, 0)


def test_plan_an_area_of_the_map():
    job = area_job(MAP, 'front')
    cfg = settings_for(PlannerConfig(), None, job)
    plan = stub_plan_coverage(job.boundary, job.obstacles, cfg, job.dock, job.dock[:2], job.drivable)
    assert plan.warnings == [PlannerConfig().no_body]
    assert plan.angle == pytest.approx(0.5)
    for x0, y0, x1, y1 in ((4, 2, 5, 3), (7, 4, 8, 5)):  # the bed and the pond
        assert not [p for p in plan.poses if x0 < p.x < x1 and y0 < p.y < y1]


# settings --------------------------------------------------------------------------------------------------


def test_openmower_settings():
    assert openmower_settings(None, 0.22) == {}
    s = openmower_settings({'/mower_logic/tool_width': 0.13, '/mower_logic/outline_count': 3,
                            'outline_overlap_count': 1, 'outline_offset': -0.05, 'mow_angle_offset': 10}, 0.22)
    # tool_width is the lane spacing: the blade stays, the overlap follows
    assert s['mower_width'] == 0.22
    cfg = PlannerConfig().with_overrides(s)
    assert cfg.lane_spacing == pytest.approx(0.13)
    assert (cfg.perimeter_passes, cfg.lane_overlap_passes, cfg.perimeter_offset) == (3, 1, -0.05)
    assert cfg.angle is None and cfg.angle_offset == pytest.approx(math.radians(10))
    # a spacing wider than the blade: no overlap
    assert openmower_settings({'tool_width': 0.3}, 0.22) == {'mower_width': 0.3, 'overlap': 0.0}
    # an absolute angle
    s = openmower_settings({'mow_angle_offset': -30, 'mow_angle_offset_is_absolute': True}, 0.22)
    assert s == {'angle': pytest.approx(math.radians(-30))}
    # the offset into -180..180 like MowingBehavior::plan_area, to the last bit: 180 is -180
    for offset, deg in ((180, -180), (-180, -180), (190, -170), (-190, 170), (725, 5), (-540, -180), (10.5, 10.5)):
        assert openmower_settings({'mow_angle_offset': offset}, 0.22) == {'angle_offset': math.radians(deg)}
    assert openmower_settings({'mow_angle_offset': 180, 'mow_angle_offset_is_absolute': True}, 0.22) == {
        'angle': -math.pi}
    # missing, NaN, negative counts, not numbers: left alone
    assert openmower_settings({'tool_width': float('nan'), 'outline_count': -1, 'outline_offset': True,
                               'mow_angle_offset': 'x', 'outline_overlap_count': None}, 0.22) == {}


def test_area_settings():
    def settings(**props):
        return area_settings(area('a', 'mow', rect(0, 0, 1, 1), **props))

    assert settings() == {}
    # NaN or no angle: automatic, -1 counts: the mower's global value
    assert settings(angle=float('nan'), outline_count=-1, outline_overlap_count=-1) == {}
    assert settings(angle=0.0, outline_count=0, outline_overlap_count=2, outline_offset=0.1) == {
        'angle': 0.0, 'perimeter_passes': 0, 'lane_overlap_passes': 2, 'perimeter_offset': 0.1}
    assert settings(angle_min=-0.2, angle_max=0.3) == {'angle_min': -0.2, 'angle_max': 0.3}
    assert settings(angle_min=-0.2) == {}  # only together
    # any planner setting by its name, over the OpenMower ones
    assert settings(outline_count=1, planner={'fill_pattern': 'concentric', 'perimeter_passes': 3}) == {
        'perimeter_passes': 3, 'fill_pattern': 'concentric'}
    with pytest.raises(PlanningError, match='planner must be an object'):
        settings(planner='concentric')
    # an angle_strategy there works the angle out, the area's angle doesn't count then
    assert settings(angle=0.5, planner={'angle_strategy': 'optimal'}) == {'angle': None, 'angle_strategy': 'optimal'}
    assert settings(angle=0.5, planner={'angle_strategy': 'optimal', 'angle': 0.2})['angle'] == 0.2
    # only the ones OpenMower doesn't know (and so doesn't send its planner)
    body = [[0.4, 0.2], [-0.1, 0.2], [-0.1, -0.2], [0.4, -0.2]]
    own = area_settings(area('a', 'mow', rect(0, 0, 1, 1), angle=0.5, outline_count=1, outline_offset=0.1,
                             angle_min=0.1, angle_max=0.4, planner={'turn_radius': 0.3, 'headland_turns': True,
                                                                    'headland_corners': 'sharp', 'turn_on_spot': True,
                                                                    'robot_outline': body}),
                        openmower=False)
    assert own == {'angle_min': 0.1, 'angle_max': 0.4, 'turn_radius': 0.3, 'headland_turns': True,
                   'headland_corners': 'sharp', 'turn_on_spot': True, 'robot_outline': body}
    cfg = PlannerConfig().with_overrides(own)
    assert cfg.headland_turns is cfg.turn_on_spot is True and cfg.headland_corners == 'sharp'
    assert cfg.robot_outline == tuple(map(tuple, body)) and cfg.body


def test_settings_for_layers():
    base = PlannerConfig(perimeter_passes=4, turn_radius=0.3)
    params = {'/mower_logic/outline_count': 3, '/mower_logic/tool_width': 0.2}
    job = area_job(MAP, 'front')  # outline_count 1, angle 0.5
    cfg = settings_for(base, params, None)
    assert (cfg.perimeter_passes, cfg.lane_spacing, cfg.turn_radius) == (3, pytest.approx(0.2), 0.3)
    cfg = settings_for(base, params, job)
    assert (cfg.perimeter_passes, cfg.angle) == (1, 0.5)
    cfg = settings_for(base, params, job, {'perimeter_passes': 2, 'angle': None, 'turn_types': 'bulb'})
    assert (cfg.perimeter_passes, cfg.angle, cfg.turn_types) == (2, None, ('bulb',))
    assert cfg.lane_spacing == pytest.approx(0.2) and cfg.turn_radius == 0.3
    assert base.perimeter_passes == 4  # not changed
    # lane_spacing_mode auto: the blade as set, not widened to tool_width; the area's or the request's where they set it
    auto = base.with_overrides({'lane_spacing_mode': 'auto', 'mower_width': 0.18})
    assert settings_for(auto, params, job).mower_width == 0.18
    assert settings_for(auto, params, job, {'mower_width': 0.25}).mower_width == 0.25
    wide = area_job({'areas': [area('a', 'mow', rect(0, 0, 10, 6), planner={'mower_width': 0.22})]}, 0)
    assert settings_for(auto, params, wide).mower_width == 0.22
    with pytest.raises(PlanningError, match='overlap'):
        settings_for(base, params, job, {'overlap': 1.5})
    for request, what in (({'colour': 'green'}, 'colour'), ({'turn_types': 5}, 'turn_types'),
                          ({'perimeter_passes': 'x'}, 'perimeter_passes'), ({'mower_width': -1}, 'mower_width'),
                          ({'fill_pattern': 3}, 'fill_pattern'), ({'angle': 'x'}, 'angle')):
        with pytest.raises(PlanningError, match=what):
            settings_for(base, params, job, request)
    # mow_angle_offset relative: the area's angle plus it. Absolute: like OpenMower the offset is the angle and
    # the area's angle doesn't count, the rest of the area and its planner object do, a request still can
    cfg = settings_for(base, dict(params, mow_angle_offset=30), job)
    assert (cfg.angle, cfg.angle_offset) == (pytest.approx(0.5 + math.radians(30)), 0.0)
    absolute = dict(params, mow_angle_offset=30, mow_angle_offset_is_absolute=True)
    cfg = settings_for(base, absolute, job)
    assert (cfg.angle, cfg.angle_offset, cfg.perimeter_passes) == (pytest.approx(math.radians(30)), 0.0, 1)
    assert settings_for(base, absolute, job, {'angle': 0.2}).angle == 0.2
    for planner, angle, strategy in (({'angle': 0.1}, 0.1, 'longest_edge'),
                                     ({'angle_strategy': 'optimal'}, None, 'optimal')):
        own = area_job({'areas': [area('a', 'mow', rect(0, 0, 10, 6), angle=0.5, angle_min=-1, angle_max=1,
                                       planner=planner)]}, 0)
        cfg = settings_for(base, absolute, own)
        assert (cfg.angle, cfg.angle_strategy, cfg.angle_min, cfg.angle_max) == (angle, strategy, -1, 1)
    # parameters that aren't any are left out
    for odd in ([1, 2], 'x', 5,
                {'tool_width': 'x', 'outline_count': [3], 'outline_offset': {}, 'mow_angle_offset': 10 ** 400}):
        assert settings_for(base, odd, None) == base
    with pytest.raises(PlanningError, match='perimeter_passes'):
        settings_for(base, {'outline_count': 300}, None)


def test_settings_for_own_settings_beat_the_mowers():
    params = {'/mower_logic/tool_width': 0.14, '/mower_logic/outline_count': 3}
    base = PlannerConfig(mower_width=0.18, perimeter_passes=-1, overlap=0.1)
    assert settings_for(base, params, None).perimeter_passes == 3
    cfg = settings_for(base, params, None, own={'perimeter_passes', 'overlap'})
    assert cfg.perimeter_passes == -1 and cfg.overlap == 0.1 and cfg.mower_width == 0.18
    # the area's own outline_count still beats them
    job = area_job({'areas': [area('lawn', 'mow', rect(0, 0, 10, 8), outline_count=2)]}, 'lawn')
    assert settings_for(base, params, job, own={'perimeter_passes'}).perimeter_passes == 2


def test_openmower_angle():
    # towards the first point more than 2 m from the first, not along the longest edge
    assert openmower_angle([(0, 4), (1, 2), (2, 0), (20, 0), (20, 12), (0, 12)]) == pytest.approx(math.atan2(-4, 2))
    assert openmower_angle([(0, 0), (1, 0), (1, 1)]) == 0.0
    assert openmower_angle([]) == 0.0


def test_settings_for_angle_like_the_node():
    """The lane angle the ROS 1 node plans with: what mower_logic sends (the area's angle or OpenMower's own,
    plus mow_angle_offset or that alone) unless the planner object has one, or the node's settings have one
    and the area doesn't."""
    lawn = [(0.0, 4.0), (2.0, 0.0), (20.0, 0.0), (20.0, 12.0), (0.0, 12.0)]
    auto = math.atan2(-4.0, 2.0)  # the first point more than 2 m away: the short slanted edge
    offset, absolute = {'mow_angle_offset': 10}, {'mow_angle_offset': 30, 'mow_angle_offset_is_absolute': True}

    def angle(base=PlannerConfig(), params=None, own_angle=False, **props):
        cfg = settings_for(base, params, area_job({'areas': [area('a', 'mow', lawn, **props)]}, 0), None, own_angle)
        return cfg.angle, cfg.angle_offset, cfg.angle_strategy

    assert angle() == (pytest.approx(auto), 0.0, 'longest_edge')
    assert angle(params=offset) == (pytest.approx(auto + math.radians(10)), 0.0, 'longest_edge')
    assert angle(params=offset, angle=0.5) == (pytest.approx(0.5 + math.radians(10)), 0.0, 'longest_edge')
    assert angle(params=absolute, angle=0.5) == (pytest.approx(math.radians(30)), 0.0, 'longest_edge')
    # the planner object's angle or strategy as it is, its angle_offset on top of the angle mower_logic sends
    assert angle(params=offset, angle=0.5, planner={'angle': 0.2}) == (0.2, 0.0, 'longest_edge')
    assert angle(params=absolute, planner={'angle_strategy': 'min_width'}) == (None, 0.0, 'min_width')
    assert angle(params=offset, angle=0.5, planner={'angle_offset': 0.1}) == (
        pytest.approx(0.5 + math.radians(10)), 0.1, 'longest_edge')
    # angle or angle_strategy in the node's settings: the area's own angle still counts
    own = PlannerConfig(angle_strategy='min_width')
    assert angle(own, offset, True) == (None, 0.0, 'min_width')
    assert angle(own, absolute, True) == (None, 0.0, 'min_width')
    assert angle(own, offset, True, angle=0.5) == (pytest.approx(0.5 + math.radians(10)), 0.0, 'min_width')
    assert angle(PlannerConfig(angle=0.3), offset, True) == (0.3, 0.0, 'longest_edge')
    assert angle(PlannerConfig(angle=0.3), offset, True, planner={'angle': 0.2}) == (0.2, 0.0, 'longest_edge')
