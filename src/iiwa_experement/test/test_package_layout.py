"""The package installs the files that the launch entry point needs."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


PACKAGE = Path(__file__).resolve().parents[1]


def test_package_declares_its_ros_dependencies():
    manifest = (PACKAGE / 'package.xml').read_text(encoding='utf-8')
    for dependency in (
        'rclpy',
        'ament_index_python',
        'webots_ros2_msgs',
        'std_srvs',
        'python3-yaml',
        'iiwa_description',
        'iiwa_bringup',
        'iiwa_config',
        'launch',
        'launch_ros',
    ):
        assert f'<exec_depend>{dependency}</exec_depend>' in manifest


def test_versioned_specifications_match_the_approved_copies():
    root = PACKAGE.parents[1]
    pairs = (
        ('docs/superpowers/specs/2026-11-08-grant-experiments.md', 'doc/experements.md'),
        ('docs/superpowers/specs/2026-11-08-grant-scenes.md', 'doc/experiment-scenes.md'),
    )
    for source, installed in pairs:
        assert (PACKAGE / installed).read_text(encoding='utf-8') == (
            root / source
        ).read_text(encoding='utf-8')


def test_expected_resources_are_installed():
    share = Path(get_package_share_directory('iiwa_experement'))
    for relative in (
        'package.xml',
        'doc/experements.md',
        'doc/experiment-scenes.md',
        'doc/README.md',
        'worlds/grant/iiwa-grant.wbt',
        'config/scenes.yaml',
        'launch/experiment.launch.py',
    ):
        path = share / relative
        assert path.is_file(), path
        assert path.stat().st_size > 0, path
