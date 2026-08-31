from setuptools import find_packages, setup

package_name = "go2w_camera_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/camera_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="robot_scene_demo maintainers",
    maintainer_email="maintainer@example.invalid",
    description="Read-only Go2-W built-in RGB ROS 2 bridge",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_bridge = go2w_camera_bridge.camera_bridge_node:main",
        ]
    },
)
