"""Launch one deterministic grant experiment in the Webots simulator."""
from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from iiwa_experement.runtime_config import (
    prepare_runtime_files,
    validate_experiment_id,
)


BRINGUP_LAUNCH = ('launch', 'iiwa.launch.py')
SCENE_SPAWNER = 'scene_spawner'


def spawner_parameters(experiment_id: int, scene_config: str) -> dict:
    """Parameters passed unchanged to the local scene_spawner node."""
    return {
        'experiment_id': int(experiment_id),
        'scene_config': str(scene_config),
    }


def runtime_actions(
    experiment_id: int,
    scene_config: str,
    setting_path: str,
    rviz: str = 'false',
    cleanup_dir: str | None = None,
) -> list:
    """Build the actions after validation has already succeeded."""
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('iiwa_bringup'), *BRINGUP_LAUNCH,
                ])
            ),
            launch_arguments={
                'simulate': 'true',
                'rviz': str(rviz),
                'setting': str(setting_path),
            }.items(),
        ),
        Node(
            package='iiwa_experement',
            executable=SCENE_SPAWNER,
            name=SCENE_SPAWNER,
            output='screen',
            parameters=[spawner_parameters(experiment_id, scene_config)],
        ),
    ]
    if cleanup_dir is not None:
        actions.append(RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(
                function=_remove_directory,
                kwargs={'directory': cleanup_dir},
            )])
        ))
    return actions


def _remove_directory(context, *args, directory: str, **kwargs):
    del context, args, kwargs
    shutil.rmtree(directory, ignore_errors=True)
    return []


def _setup(context, *args, **kwargs):
    del args, kwargs
    raw_id = LaunchConfiguration('experiment_id').perform(context)
    try:
        experiment_id = validate_experiment_id(raw_id)
    except ValueError as error:
        raise RuntimeError(f'Invalid experiment_id: {error}') from error

    scene_config = str(LaunchConfiguration('scene_config').perform(context))
    base_setting = str(LaunchConfiguration('base_setting').perform(context))
    rviz = str(LaunchConfiguration('rviz').perform(context)).lower()

    package_share = Path(get_package_share_directory('iiwa_experement'))
    description_share = Path(get_package_share_directory('iiwa_description'))
    world_template = package_share / 'worlds' / 'grant' / 'iiwa-grant.wbt'

    output_dir = Path(tempfile.mkdtemp(prefix='iiwa_experement_'))
    try:
        world_path, setting_path = prepare_runtime_files(
            base_setting=base_setting,
            world_template=world_template,
            description_share=description_share,
            output_dir=output_dir,
        )
    except Exception:
        shutil.rmtree(output_dir, ignore_errors=True)
        raise

    print(f'[iiwa_experement] experiment {experiment_id}: {world_path}')
    return runtime_actions(
        experiment_id=experiment_id,
        scene_config=scene_config,
        setting_path=str(setting_path),
        rviz=rviz,
        cleanup_dir=str(output_dir),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'experiment_id',
            default_value='',
            description='Experiment number 1..20; required.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='true = start RViz together with Webots.',
        ),
        DeclareLaunchArgument(
            'scene_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('iiwa_experement'), 'config', 'scenes.yaml',
            ]),
            description='Scenario manifest; passed unchanged to scene_spawner.',
        ),
        DeclareLaunchArgument(
            'base_setting',
            default_value=PathJoinSubstitution([
                FindPackageShare('iiwa_config'), 'config', 'setting.yaml',
            ]),
            description='Base settings used to build the temporary setting.yaml.',
        ),
        OpaqueFunction(function=_setup),
    ])
