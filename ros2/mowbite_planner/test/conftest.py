"""The marks of the planner's tests."""


def pytest_configure(config):
    config.addinivalue_line('markers', 'real_planner: plans with plan_coverage itself where the file stands it in')
    config.addinivalue_line('markers', 'matrix: the invariants over the matrix of shapes, bodies and settings (heavy)')
