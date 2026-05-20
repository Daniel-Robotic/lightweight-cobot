import argparse
import sys

from cobot.commands import docker_setup as cmd_docker_setup
from cobot.commands import doc_setup as cmd_doc_setup
from cobot.commands import robot_setup as cmd_robot_setup
from cobot.commands import setup as cmd_setup

# Command groups shown in --help output.
# Add new commands here when introducing other categories.
_GROUPS = [
    ("Setup", [
        ("setup",        "run doc-setup + docker-setup + robot-setup in one go"),
        ("docker-setup", "build or pull Docker images for KUKA iiwa7"),
        ("doc-setup",    "deploy or stop the MkDocs documentation server"),
        ("robot-setup",  "configure cobot-setting.yaml interactively"),
    ]),
]

_DESCRIPTION = "Lightweight Cobot"


class _GroupedHelpAction(argparse.Action):
    def __init__(self, option_strings, dest, default=None, required=False, help=None):
        super().__init__(
            option_strings=option_strings,
            dest=dest,
            nargs=0,
            default=default,
            required=required,
            help=help,
        )

    def __call__(self, parser, namespace, values, option_string=None):
        print(f"usage: cobot [-h] <command> ...\n")
        print(f"{_DESCRIPTION}\n")
        for group_title, commands in _GROUPS:
            print(f"{group_title} commands:")
            for cmd, help_text in commands:
                print(f"  {cmd:<22} {help_text}")
            print()
        print("options:")
        print("  -h, --help             show this help message and exit")
        parser.exit()


def main():
    parser = argparse.ArgumentParser(
        prog="cobot",
        description=_DESCRIPTION,
        add_help=False,
    )
    parser.add_argument(
        "-h", "--help",
        action=_GroupedHelpAction,
        default=argparse.SUPPRESS,
        help="show this help message and exit",
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
    cmd_robot_setup.register(subparsers)
