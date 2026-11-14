from glob import glob

from setuptools import find_packages, setup

package_name = "pick_place_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Finds the object in the wrist camera's depth image (omniverse_hookup step 5).",
    license="MIT",
    entry_points={"console_scripts": ["object_locator = pick_place_ros.object_locator:main"]},
)
