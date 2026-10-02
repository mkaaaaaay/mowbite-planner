import {describe, expect, it} from 'vitest';
import {readPlan} from './areaPlan';

describe('readPlan', () => {
  it('reads the planner answer as ROS gives it', () => {
    const pose = (x: number, y: number) => ({pose: {position: {x, y}}});
    const plan = readPlan({
      paths: [
        {is_outline: 1, path: {poses: [pose(0, 0), pose(1, 0), pose(1, 1)]}},
        {is_outline: 0, path: {poses: [pose(0.2, 0.2), pose(0.8, 0.2), pose(0.8, 0.4)]}},
      ],
    });
    expect(plan.loops).toEqual([[{x: 0, y: 0}, {x: 1, y: 0}, {x: 1, y: 1}]]);
    expect(plan.stripes).toHaveLength(2);
  });

  it('also takes plain point lists', () => {
    const plan = readPlan([{outline: true, points: [[0, 0], [2, 0], [2, 2]]}]);
    expect(plan.loops[0][1]).toEqual({x: 2, y: 0});
  });
});
