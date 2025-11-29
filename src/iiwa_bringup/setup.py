import os

from setuptools import setup

package_name = "iiwa_bringup"

# Собираем список пакетов: основной + вложенные (если они есть).
packages = [package_name]
# Если есть подпапка utils с __init__.py — зарегистрируем её как iiwa_bringup.utils
if os.path.isdir("utils") and os.path.isfile(os.path.join("utils", "__init__.py")):
    packages.append(f"{package_name}.utils")

# Соответствие имени пакета -> директория на диске.
# iiwa_bringup -> текущая папка '.'
# iiwa_bringup.utils -> ./utils
package_dir = {
    package_name: ".",
}
if f"{package_name}.utils" in packages:
    package_dir[f"{package_name}.utils"] = os.path.join(".", "utils")


def data_files_from_tree(src_dir: str, dst_root: str) -> list:
    entries = []
    if not os.path.isdir(src_dir):
        return entries
    EXCLUDE_DIRS = {".git", "__pycache__", ".pytest_cache", ".idea"}
    EXCLUDES = {".DS_Store", "Thumbs.db"}
    for root, dirs, files in os.walk(src_dir):
        dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
        file_list = [os.path.join(root, f) for f in files if f not in EXCLUDES]
        if not file_list:
            continue
        rel = os.path.relpath(root, src_dir)
        dst_dir = os.path.join(dst_root, rel) if rel != "." else dst_root
        entries.append((dst_dir, file_list))
    return entries


data_files = [(f"share/{package_name}", ["package.xml"])]
data_files += data_files_from_tree("launch", f"share/{package_name}/launch")
data_files += data_files_from_tree("config", f"share/{package_name}/config")
data_files += data_files_from_tree("resource", f"share/{package_name}/resource")

setup(
    name=package_name,
    version="0.0.1",
    packages=packages,
    package_dir=package_dir,
    include_package_data=True,
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Grabar Daniil",
    maintainer_email="grabardm@ml-dev.ru",
    description="iiwa bringup package",
    license="Apache-2.0",
)
