import os
from setuptools import find_packages, setup

EXCLUDES = {'.DS_Store', 'Thumbs.db'}
EXCLUDE_DIRS = {'__pycache__', '.pytest_cache', '.git', '.idea'}

def data_files_from_tree(src_dir: str, dst_root: str) -> list:
    """
    Собирает data_files в формате, подходящем для setuptools.
    Возвращает список пар (dst_path, [file1, file2, ...]).
    """
    entries = []

    for root, dirs, files in os.walk(src_dir):
        # исключаем служебные директории
        dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]

        file_list = [os.path.join(root, f) for f in files if f not in EXCLUDES]
        if not file_list:
            continue

        rel = os.path.relpath(root, src_dir)
        dst_dir = os.path.join(dst_root, rel) if rel != '.' else dst_root
        entries.append((dst_dir, file_list))

    return entries

package_name = 'iiwa_bringup'

resource_intsall_root = f"share/{package_name}/resource"
launch_intsall_root = f"share/{package_name}/launch"
config_intsall_root = f"share/{package_name}/config"


data_files = [
    ('share/' + package_name, ['package.xml']),
]

data_files += data_files_from_tree('resource', resource_intsall_root)
data_files += data_files_from_tree('launch', launch_intsall_root)
data_files += data_files_from_tree('config', config_intsall_root)



setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grabar Daniil',
    maintainer_email='grabardm@ml-dev.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
