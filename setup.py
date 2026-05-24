from setuptools import setup, find_packages

setup(
    name="lightweight-cobot",
    version="2026.05.31",
    description="CLI tool for installing, configuring and managing the ROS 2 cobot workspace",
    packages=find_packages(),
    python_requires=">=3.11",
    install_requires=[
        "textual",
        "ruamel.yaml",
    ],
    entry_points={
        "console_scripts": [
            "cobot=cobot.cli:main",
        ],
    },
)
