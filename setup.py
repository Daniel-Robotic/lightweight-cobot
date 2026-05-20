from setuptools import setup, find_packages

setup(
    name="lightweight-cobot",
    version="0.1.0",
    description="Lightweight Cobot — KUKA iiwa7 ROS2 Control Framework",
    packages=find_packages(),
    python_requires=">=3.11",
    install_requires=[
        "textual",
    ],
    entry_points={
        "console_scripts": [
            "cobot=cobot.cli:main",
        ],
    },
)
