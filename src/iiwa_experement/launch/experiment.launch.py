"""Launch one deterministic grant experiment in the Webots simulator."""
from __future__ import annotations

import shutil
import sys
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.utils import controller_url_prefix

from iiwa_experement.runtime_config import (
    DEFAULT_START_DELAY_SEC,
    DEFAULT_TRACE_DIR,
    prepare_runtime_files,
    validate_experiment_id,
    validate_mode,
    validate_start_delay,
    validate_trace_dir,
)


BRINGUP_LAUNCH = ('launch', 'iiwa.launch.py')
SCENE_SPAWNER = 'scene_spawner'
SUPERVISOR = 'grant_supervisor'
SUPERVISOR_ROBOT = 'GrantAttachmentSupervisor'


def spawner_parameters(experiment_id: int, scene_config: str) -> dict:
    """Parameters passed unchanged to the local scene_spawner node."""
    return {
        'experiment_id': int(experiment_id),
        'scene_config': str(scene_config),
    }


def supervisor_parameters(
    experiment_id: int,
    scene_config: str,
    geometry_path: str,
    read_only: bool,
) -> dict:
    return {
        'experiment_id': int(experiment_id),
        'scene_config': str(scene_config),
        'geometry_path': str(geometry_path),
        'read_only': bool(read_only),
    }


def runner_parameters(
    experiment_id: int,
    scene_config: str,
    geometry_path: str,
    start_delay_sec: float,
    trace_dir: str,
) -> dict:
    return {
        'experiment_id': int(experiment_id),
        'scene_config': str(scene_config),
        'geometry_path': str(geometry_path),
        'start_delay_sec': float(start_delay_sec),
        'trace_dir': str(trace_dir),
    }


def supervisor_process(
    experiment_id: int,
    scene_config: str,
    geometry_path: str,
    *,
    read_only: bool,
) -> ExecuteProcess:
    """Start the Webots supervisor beside Webots.

    The process must exist before the scene is spawned: Webots waits for all
    ``<extern>`` controllers, so a later start would deadlock the simulation.
    """
    parameters = supervisor_parameters(
        experiment_id, scene_config, geometry_path, read_only,
    )
    arguments = []
    for key, value in parameters.items():
        text = str(value).lower() if isinstance(value, bool) else str(value)
        arguments.extend(['-p', f'{key}:={text}'])
    return ExecuteProcess(
        cmd=[
            sys.executable, '-m', 'iiwa_experement.grant_supervisor',
            '--ros-args', *arguments,
        ],
        name=SUPERVISOR,
        output='screen',
        additional_env={
            'WEBOTS_CONTROLLER_URL': controller_url_prefix('1234') + SUPERVISOR_ROBOT,
            'WEBOTS_HOME': get_package_prefix('webots_ros2_driver'),
        },
    )


def runtime_actions(
    experiment_id: int,
    scene_config: str,
    setting_path: str,
    rviz: str = 'false',
    cleanup_dir: str | None = None,
    *,
    mode: str = 'view',
    start_delay_sec: float = DEFAULT_START_DELAY_SEC,
    trace_dir: str = DEFAULT_TRACE_DIR,
    geometry_path: str | None = None,
) -> list:
    """Build the actions after validation has already succeeded."""
    package_share = Path(get_package_share_directory('iiwa_experement'))
    if geometry_path is None:
        geometry_path = str(package_share / 'config' / 'geometry.yaml')
    read_only = mode != 'execute'
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
        supervisor_process(
            experiment_id,
            scene_config,
            str(geometry_path),
            read_only=read_only,
        ),
    ]
    if mode == 'execute':
        actions.append(Node(
            package='iiwa_experement',
            executable='reference_runner',
            name='reference_runner',
            output='screen',
            parameters=[
                runner_parameters(
                    experiment_id, scene_config, geometry_path,
                    start_delay_sec, trace_dir,
                ),
                {'use_sim_time': True},
            ],
        ))
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

    try:
        mode = validate_mode(LaunchConfiguration('mode').perform(context))
    except ValueError as error:
        raise RuntimeError(f'Invalid mode: {error}') from error
    try:
        start_delay_sec = validate_start_delay(
            LaunchConfiguration('start_delay_sec').perform(context)
        )
    except ValueError as error:
        raise RuntimeError(f'Invalid start_delay_sec: {error}') from error
    try:
        trace_dir = validate_trace_dir(
            LaunchConfiguration('trace_dir').perform(context)
        )
    except ValueError as error:
        raise RuntimeError(f'Invalid trace_dir: {error}') from error

    scene_config = str(LaunchConfiguration('scene_config').perform(context))
    base_setting = str(LaunchConfiguration('base_setting').perform(context))
    rviz = str(LaunchConfiguration('rviz').perform(context)).lower()
    geometry_path = str(
        Path(get_package_share_directory('iiwa_experement'))
        / 'config'
        / 'geometry.yaml'
    )

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
            moveit_cpp_path=package_share / 'config' / 'moveit_cpp.yaml',
        )
    except Exception:
        shutil.rmtree(output_dir, ignore_errors=True)
        raise

    print(
        f'[iiwa_experement] experiment {experiment_id} mode={mode} '
        f'delay={start_delay_sec:g}s: {world_path}'
    )
    return runtime_actions(
        experiment_id=experiment_id,
        scene_config=scene_config,
        setting_path=str(setting_path),
        rviz=rviz,
        cleanup_dir=str(output_dir),
        mode=mode,
        start_delay_sec=start_delay_sec,
        trace_dir=trace_dir,
        geometry_path=geometry_path,
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'experiment_id',
            default_value='',
            description='Experiment number 1..20; required.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='view',
            description=(
                'view = open the prepared scene without motion; '
                'execute = run the reference sequence after start_delay_sec.'
            ),
        ),
        DeclareLaunchArgument(
            'start_delay_sec',
            default_value=f'{DEFAULT_START_DELAY_SEC:g}',
            description='Simulation-time delay before the first execute motion.',
        ),
        DeclareLaunchArgument(
            'trace_dir',
            default_value=DEFAULT_TRACE_DIR,
            description='Directory for one JSONL trace per run.',
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
