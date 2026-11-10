from glob import glob

from setuptools import find_packages, setup

package_name = 'iiwa_llm_plan_executor'

setup(
    name=package_name,
    version='2026.11.10',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/doc', glob('doc/*.md') + glob('docs/*.md')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='grabardm@ml-dev.ru',
    description='Validate and execute externally supplied Qwen3 plan graphs in Webots',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'llm_plan_executor = iiwa_llm_plan_executor.node:main',
        ],
    },
)
