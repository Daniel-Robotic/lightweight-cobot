from setuptools import find_packages, setup

package_name = 'iiwa_web'

setup(
    name=package_name,
    version='2026.5.31',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn[standard]'
    ],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='grabardm@ml-dev.ru',
    description='Web interface for monitoring and remote control of the cobot via browser',
    license='Apache-2.0',
    extras_require={
    },
    entry_points={
        'console_scripts': [
            'iiwa_web_server = iiwa_web.main:main',
        ],
    },
)
