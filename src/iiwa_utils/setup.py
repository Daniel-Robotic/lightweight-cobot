from setuptools import find_packages, setup

package_name = 'iiwa_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='grabardm@ml-dev.ru',
    description='Вспомогательные файлы для работы всей системы',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'object_spawner = iiwa_utils.object_spawner:main',
            "motion_planning_test = iiwa_utils.motion_planing_test:main"
        ],
    },
)
