from setuptools import setup, find_packages

setup(
    name="lightweight-cobot",
    version="0.1.0",
    description="Lightweight Cobot",
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
