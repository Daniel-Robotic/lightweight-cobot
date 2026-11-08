"""The package installs the files that the launch entry point needs."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


PACKAGE = Path(__file__).resolve().parents[1]


def test_package_installs_both_run_mode_executables():
    text = (PACKAGE / 'setup.py').read_text(encoding='utf-8')
    assert 'grant_supervisor = iiwa_experement.grant_supervisor:main' in text
    assert 'reference_runner = iiwa_experement.reference_runner:main' in text


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


def test_package_documentation_points_at_the_reference_specification():
    root = PACKAGE.parents[1]
    reference = (
        root
        / 'docs'
        / 'superpowers'
        / 'specs'
        / '2026-11-08-grant-reference-control-design.md'
    )
    assert reference.is_file()
    for relative in ('doc/experements.md', 'doc/experiment-scenes.md'):
        text = (PACKAGE / relative).read_text(encoding='utf-8')
        assert '2026-11-08-grant-reference-control-design.md' in text
        assert 'tighten' not in text.lower() or 'не затяг' in text.lower()
        assert 'seat_screw' not in text


def test_expected_resources_are_installed():
    share = Path(get_package_share_directory('iiwa_experement'))
    for relative in (
        'package.xml',
        'doc/experements.md',
        'doc/experiment-scenes.md',
        'doc/README.md',
        'doc/reference-control-qa.md',
        'worlds/grant/iiwa-grant.wbt',
        'config/scenes.yaml',
        'config/geometry.yaml',
        'config/moveit_cpp.yaml',
        'launch/experiment.launch.py',
    ):
        path = share / relative
        assert path.is_file(), path
        assert path.stat().st_size > 0, path
