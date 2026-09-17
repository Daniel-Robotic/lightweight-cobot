"""TCP must be at the active tool's working end in the visual model."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from iiwa_utils.tool_manager import _write_xacro, load_registry

ROOT = Path(__file__).resolve().parents[2]


def test_patron_tcp_is_on_the_distal_face_of_its_mesh():
    tool = ET.parse(ROOT / 'iiwa_description/urdf/tools/patron.xacro').getroot()
    joint = tool.find("joint[@name='patron_tcp']")
    assert joint.find('parent').attrib['link'] == 'patron'
    assert joint.find('child').attrib['link'] == 'tcp'
    tcp = np.fromstring(joint.find('origin').attrib['xyz'], sep=' ')
    mesh = ET.parse(ROOT / 'iiwa_description/resource/meshes/tools/patron.dae').getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    assert float(mesh.find('c:asset/c:unit', ns).attrib['meter']) == 1.
    geometry = mesh.find('.//c:mesh', ns)
    ref = geometry.find("c:vertices/c:input[@semantic='POSITION']", ns).attrib['source'][1:]
    values = geometry.find(f"c:source[@id='{ref}']/c:float_array", ns)
    points = np.fromstring(values.text, sep=' ').reshape(-1, 3)
    np.testing.assert_allclose(tcp, [0., 0., points[:, 2].max()], atol=1e-6)


def test_without_tool_tcp_coincides_with_flange(tmp_path):
    registry = load_registry(ROOT / 'iiwa_config/config/tools.yaml')
    assert registry['none']['tcp_link'] == 'tcp'
    output = tmp_path / 'tool.xacro'
    _write_xacro(registry['none'], output)
    joint = ET.parse(output).getroot().find("joint[@name='link_ee_to_tcp']")
    assert joint.find('parent').attrib['link'] == 'link_ee'
    assert joint.find('child').attrib['link'] == 'tcp'
    np.testing.assert_array_equal(np.fromstring(joint.find('origin').attrib['xyz'], sep=' '), [0, 0, 0])
