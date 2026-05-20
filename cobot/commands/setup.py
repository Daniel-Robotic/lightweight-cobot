import argparse

from cobot.commands.doc_setup import run as _doc_setup
from cobot.commands.docker_setup import run as _docker_setup


def register(subparsers):
    p = subparsers.add_parser("setup", help="First-time project setup")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    _doc_setup(args)
    _docker_setup(args)
