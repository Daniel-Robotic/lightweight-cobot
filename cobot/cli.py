import argparse
import importlib
import sys

from cobot.commands import setup as cmd_setup


def main():
    parser = argparse.ArgumentParser(
        prog="cobot",
        description="Lightweight Cobot - Cobot ROS2 Control Framework",
    )
    subparsers = parser.add_subparsers(dest="command", metavar="<command>")
    subparsers.required = True

    # Регистрируем подкоманды — каждый модуль в cobot/commands/ описывает свою
    _register_commands(subparsers)

    args = parser.parse_args()
    args.func(args)


def _register_commands(subparsers):
    cmd_setup.register(subparsers)
