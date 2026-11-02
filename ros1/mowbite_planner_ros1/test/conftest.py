"""The marks of the node's tests."""


def pytest_configure(config):
    config.addinivalue_line('markers', 'real_planner: plans with plan_coverage itself, not the stand-in (stubplan.py)')
