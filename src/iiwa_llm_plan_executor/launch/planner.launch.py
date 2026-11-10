"""Launch one Webots scene with the scene context and the graph executor.

The experiment only selects the initial scene.  The reference runner is never
started in this mode; the graph arrives from outside through
``/iiwa_llm_plan_executor/submit``.
"""
from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from iiwa_experement.runtime_config import (
    validate_experiment_id,
    validate_mode,
)


EXPERIMENT_LAUNCH = ('launch', 'experiment.launch.py')
PLANNER_MODE = 'planner'
DEFAULT_CONFIG = {
    'start_delay_sec': 3.0,
    'scene_max_age_sec': 2.0,
    'max_steps': 512,
    'scene_freshness_sec': 2.0,
    'close_inspection_objects': ['cap'],
    'top_camera_topic': '/d455_top/image_raw/image_color',
    'hand_camera_topic': '/camera/color/image_color',
    'frame_timeout_sec': 2.0,
    'trace_dir': '~/.ros/iiwa_llm_plan_executor/runs',
    'planning_group': 'iiwa_arm',
    'pose_link': 'tcp',
}


def load_config(path: Path | str | None) -> dict:
    data = dict(DEFAULT_CONFIG)
    if path is not None:
        source = Path(path)
        if source.is_file():
            loaded = yaml.safe_load(source.read_text(encoding='utf-8')) or {}
            if isinstance(loaded, dict):
                data.update(loaded)
    return data


def validate_arguments(
    experiment_id_raw: str,
    mode_raw: str,
    simulate_raw: str,
) -> tuple[int, str]:
    """Validate everything before Webots is started."""
    experiment_id = validate_experiment_id(experiment_id_raw)
    mode = validate_mode(mode_raw)
    if mode != PLANNER_MODE:
        raise ValueError(
            f'mode must be {PLANNER_MODE!r} for the graph executor, got {mode!r}'
        )
    simulate = str(simulate_raw).strip().lower()
    if simulate not in ('true', '1', 'yes'):
        raise ValueError(
            'this executor stage only supports Webots (simulate:=true); '
            'the physical FRI robot is not connected'
        )
    return experiment_id, mode


def moveit_parameters(base_setting: str) -> list:
    """Parameters MoveItPy reads from the process-wide ROS arguments.

    MoveItPy creates its own node, but ROS 2 applies the process command-line
    ``-p`` arguments to every node in the process.  The same parameter set is
    used by the existing move_to_pose_server in iiwa_bringup.
    """
    from iiwa_utils import converter, setting_loader
    from iiwa_utils.converter import effective_joint_limits
    from moveit_configs_utils import MoveItConfigsBuilder

    settings = setting_loader.build_settings(base_setting, check_files=True)
    moveit_configs = (
        MoveItConfigsBuilder('iiwa7', package_name='iiwa_config')
        .robot_description(
            file_path=settings.robot.description,
            mappings={
                'initial_positions_file': settings.controller.moveit.initial_positions,
                'joint_limits_file': settings.controller.moveit.joint_limits,
            },
        )
        .robot_description_semantic(file_path=settings.controller.moveit.srdf)
        .robot_description_kinematics(file_path=settings.controller.moveit.kinematics)
        .joint_limits(file_path=settings.controller.moveit.joint_limits)
        .pilz_cartesian_limits(file_path=settings.controller.moveit.pilz_limits)
        .trajectory_execution(file_path=settings.controller.moveit.moveit_controllers)
        .moveit_cpp(file_path=settings.controller.moveit.moveit_cpp)
        .to_moveit_configs()
    )
    moveit_configs.joint_limits['robot_description_planning'] = (
        effective_joint_limits(moveit_configs.joint_limits['robot_description_planning'])
    )
    configuration = moveit_configs.to_dict()
    # The executor checks every planned point itself.  The Ruckig smoothing
    # chain can overshoot a joint limit on a valid OMPL path and then the
    # custom LimitSplineDynamics adapter rejects the whole plan.  Keep the
    # deterministic rest-boundary/time-parameterization and validation
    # adapters for this process; move_group keeps the full pipeline.
    ompl = configuration.get('ompl')
    if isinstance(ompl, dict):
        ompl['response_adapters'] = [
            adapter for adapter in ompl.get('response_adapters', [])
            if adapter not in (
                'default_planning_response_adapters/AddRuckigTrajectorySmoothing',
                'iiwa_planning/LimitSplineDynamics',
            )
        ]
    robot_description = converter.load_robot_description(
        settings.robot.description,
        settings.robot.name,
        {
            'simulate': 'true',
            'initial_positions_file': settings.controller.moveit.initial_positions,
            'joint_limits_file': settings.controller.moveit.joint_limits,
        },
    )
    return [
        configuration,
        {'robot_description': robot_description},
        {'use_sim_time': True},
    ]


def runtime_actions(
    *,
    experiment_id: int,
    scene_config: str,
    base_setting: str,
    rviz: str,
    config: dict,
    moveit_config_parameters: list | None = None,
) -> list:
    experiment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('iiwa_experement'), *EXPERIMENT_LAUNCH,
        ])),
        launch_arguments={
            'experiment_id': str(experiment_id),
            'mode': PLANNER_MODE,
            'scene_config': str(scene_config),
            'base_setting': str(base_setting),
            'rviz': str(rviz),
        }.items(),
    )
    scene_context = Node(
        package='iiwa_scene_context',
        executable='scene_context',
        name='iiwa_scene_context',
        output='screen',
        parameters=[{
            'experiment_id': int(experiment_id),
            'scene_config': str(scene_config),
            'freshness_sec': float(config.get('scene_freshness_sec', 2.0)),
            'use_sim_time': True,
        }],
    )
    executor = Node(
        package='iiwa_llm_plan_executor',
        executable='llm_plan_executor',
        name='iiwa_llm_plan_executor',
        output='screen',
        parameters=[
            *(moveit_config_parameters or []),
            {
                'experiment_id': int(experiment_id),
                'scene_config': str(scene_config),
                'start_delay_sec': float(config.get('start_delay_sec', 3.0)),
                'scene_max_age_sec': float(config.get('scene_max_age_sec', 2.0)),
                'max_steps': int(config.get('max_steps', 512)),
                'close_inspection_objects': list(
                    config.get('close_inspection_objects', ['cap'])
                ),
                'top_camera_topic': str(
                    config.get('top_camera_topic', '/d455_top/image_raw/image_color')
                ),
                'hand_camera_topic': str(
                    config.get('hand_camera_topic', '/camera/color/image_color')
                ),
                'frame_timeout_sec': float(config.get('frame_timeout_sec', 2.0)),
                'trace_dir': str(
                    config.get('trace_dir', '~/.ros/iiwa_llm_plan_executor/runs')
                ),
                'planning_group': str(config.get('planning_group', 'iiwa_arm')),
                'pose_link': str(config.get('pose_link', 'tcp')),
                'use_sim_time': True,
            },
        ],
    )
    return [experiment, scene_context, executor]


def _setup(context, *args, **kwargs):
    del args, kwargs
    raw_id = LaunchConfiguration('experiment_id').perform(context)
    raw_mode = LaunchConfiguration('mode').perform(context)
    raw_simulate = LaunchConfiguration('simulate').perform(context)
    try:
        experiment_id, _ = validate_arguments(raw_id, raw_mode, raw_simulate)
    except ValueError as error:
        raise RuntimeError(f'Invalid planner launch arguments: {error}') from error
    config_path = LaunchConfiguration('config').perform(context)
    config = load_config(config_path)
    base_setting = str(LaunchConfiguration('base_setting').perform(context))
    moveit_config_parameters = moveit_parameters(base_setting)
    print(
        f'[iiwa_llm_plan_executor] experiment {experiment_id} planner mode: '
        f'no reference sequence; waiting for an external graph'
    )
    return runtime_actions(
        experiment_id=experiment_id,
        scene_config=str(LaunchConfiguration('scene_config').perform(context)),
        base_setting=base_setting,
        rviz=str(LaunchConfiguration('rviz').perform(context)).lower(),
        config=config,
        moveit_config_parameters=moveit_config_parameters,
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'experiment_id',
            default_value='',
            description='Experiment 1..20; selects only the initial scene.',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='planner',
            description='Must remain planner for externally supplied graphs.',
        ),
        DeclareLaunchArgument(
            'simulate',
            default_value='true',
            description='Must remain true; the physical robot is not connected.',
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
            description='Scenario manifest used to build the initial scene.',
        ),
        DeclareLaunchArgument(
            'base_setting',
            default_value=PathJoinSubstitution([
                FindPackageShare('iiwa_config'), 'config', 'setting.yaml',
            ]),
            description='Base settings for the temporary setting.yaml.',
        ),
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution([
                FindPackageShare('iiwa_llm_plan_executor'),
                'config',
                'planner.yaml',
            ]),
            description='Executor defaults: delays, limits, cameras, logging.',
        ),
        OpaqueFunction(function=_setup),
    ])
