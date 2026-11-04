"""The marks of the tests."""


def pytest_configure(config):
    config.addinivalue_line('markers', 'real_planner: plans with plan_coverage itself where the file stands it in')
