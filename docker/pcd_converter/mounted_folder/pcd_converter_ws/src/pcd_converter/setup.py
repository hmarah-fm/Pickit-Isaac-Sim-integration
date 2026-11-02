from glob import glob
from setuptools import find_packages, setup

package_name = "pcd_converter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/pcd_converter"]),
        ("share/pcd_converter", ["package.xml"]),
        ("share/pcd_converter/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Robotics Team",
    maintainer_email="robotics@example.invalid",
    description="Zivid RGB/XYZ to Pickit virtual-camera bridge.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "zivid_to_simulated_camera = pcd_converter.zivid_to_simulated_camera:main",
        ],
    },
)
