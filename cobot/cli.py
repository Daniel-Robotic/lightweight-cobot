import argparse
import importlib
import sys

from cobot.commands import docker_setup as cmd_docker_setup
from cobot.commands import doc_setup as cmd_doc_setup
from cobot.commands import setup as cmd_setup


def main():
    parser = argparse.ArgumentParser(
        prog="cobot",
        description="Lightweight Cobot - Cobot ROS2 Control Framework",
    )
    subparsers = parser.add_subparsers(dest="command", metavar="<command>")
    subparsers.required = True

    _register_commands(subparsers)

    args = parser.parse_args()
    args.func(args)


def _register_commands(subparsers):
    cmd_setup.register(subparsers)
    cmd_docker_setup.register(subparsers)
    cmd_doc_setup.register(subparsers)
