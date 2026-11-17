from setuptools import find_packages, setup

package_name = 'mowbite_planner_os'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/examples', ['examples/calib-evaluate.json']),
        ('share/' + package_name + '/api', ['api/planner.schema.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkaaaaaay',
    maintainer_email='mkaaaaaay@users.noreply.github.com',
    description='The MowBite Planner on MowBite OS',
    license='PolyForm-Noncommercial-1.0.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mowbite_planner_os_node = mowbite_planner_os.node:main',
        ],
    },
)
