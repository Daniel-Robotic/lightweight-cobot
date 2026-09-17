"""TCP teleoperation exists only in a simulated, explicitly enabled launch."""
from dataclasses import asdict
import tempfile
import xml.etree.ElementTree as ET

import yaml
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController


def make_tcp_gizmo_nodes(settings, common_params, simulate):
    cfg = settings.digital_twin.tcp_gizmo
    if not simulate or not cfg.enabled:
        return []

    # The robot's Webots placement maps base_link into the Webots world.
    robot = ET.Element('robot', name='tcp_gizmo')
    ET.SubElement(robot, 'link', name='gizmo_base')
    plugin = ET.SubElement(ET.SubElement(robot, 'webots'), 'plugin',
                           type='iiwa_utils.tcp_gizmo.WebotsTcpGizmo')
    for key, value in {
        'translation': settings.digital_twin.webots.transform,
        'rotation': settings.digital_twin.webots.rotation,
        'frame': 'base_link',
        'state_timeout': cfg.state_timeout,
    }.items():
        ET.SubElement(plugin, key).text = str(value)
    with tempfile.NamedTemporaryFile(mode='w', prefix='iiwa_tcp_gizmo_', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump({'/**': {'ros__parameters': {
            'robot_description': ET.tostring(robot, encoding='unicode'),
            'set_robot_state_publisher': False,
            'use_sim_time': True,
        }}}, stream)
        driver_params = stream.name

    servo = {
        'move_group_name': settings.planning.planning_group,
        'publish_period': cfg.publish_period,
        'max_expected_latency': max(0.1, 3 * cfg.publish_period),
        'command_in_type': 'speed_units',
        'scale': {'linear': cfg.max_linear_speed, 'rotational': cfg.max_angular_speed},
        'incoming_command_timeout': cfg.command_timeout,
        'is_primary_planning_scene_monitor': False,
        'monitored_planning_scene_topic': '/monitored_planning_scene',
        'joint_topic': '/joint_states',
        'check_collisions': True,
        'check_octomap_collisions': False,
        'publish_joint_positions': True,
        'publish_joint_velocities': False,
        'use_smoothing': True,
        'joint_limit_margins': [0.05],
    }
    return [
        Node(package='iiwa_planning', executable='tcp_gizmo', name='tcp_gizmo',
             output='screen', parameters=[*common_params, {
                 **asdict(cfg), 'simulation': True,
                 # Tool xacros define tcp at the working end; without a tool
                 # it coincides with link_ee. General pose commands may use a
                 # different planning.pose_link, but the handle controls TCP.
                 'pose_link': 'tcp',
                 'planning_group': settings.planning.planning_group,
                 'default_frame': 'base_link', 'moveit_servo': servo,
             }]),
        Node(package='iiwa_utils', executable='tcp_gizmo_spawner', output='screen'),
        WebotsController(robot_name='tcp_gizmo',
                         parameters=[{'use_sim_time': True}, driver_params]),
    ]
