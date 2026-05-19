import argparse


def register(subparsers):
    p = subparsers.add_parser("setup", help="First-time project setup")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    print("cobot setup — TODO: implement setup logic here")
