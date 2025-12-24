from setuptools import find_packages, setup

package_name = 'iiwa_object_spawner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Даниил Грабарь',
    maintainer_email='grabardm@ml-dev.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'object_spawner = iiwa_object_spawner.object_spawner:main',
            "motion_planning_test = iiwa_object_spawner.motion_planing_test:main"
        ],
    },
)
