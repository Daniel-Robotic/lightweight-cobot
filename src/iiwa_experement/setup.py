from glob import glob

from setuptools import find_packages, setup

package_name = 'iiwa_experement'

setup(
    name=package_name,
    version='2026.11.8',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/doc', glob('doc/*.md')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds/grant', glob('worlds/grant/*.wbt')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='grabardm@ml-dev.ru',
    description='Deterministic Webots scenes for the twenty KUKA iiwa 7 grant experiments',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scene_spawner = iiwa_experement.scene_spawner:main',
            'grant_supervisor = iiwa_experement.grant_supervisor:main',
        ],
    },
)
