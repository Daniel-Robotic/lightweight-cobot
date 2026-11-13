from glob import glob

from setuptools import find_packages, setup

package_name = 'iiwa_scene_context'

setup(
    name=package_name,
    version='2026.11.10',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/doc', glob('docs/*.md')),
        ('share/' + package_name + '/doc', glob('docs/*.json')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='grabardm@ml-dev.ru',
    description='Live scene snapshot and scene_json contract for externally supplied planner graphs',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scene_context = iiwa_scene_context.node:main',
        ],
    },
)
