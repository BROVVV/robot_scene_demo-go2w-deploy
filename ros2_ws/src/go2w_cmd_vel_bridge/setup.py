from setuptools import find_packages, setup

package_name = "go2w_cmd_vel_bridge"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="robot_scene_demo maintainers",
    maintainer_email="maintainer@example.invalid",
    description="Go2-W Twist to leased MotionCommand adapter",
    license="Apache-2.0",
    entry_points={"console_scripts": ["bridge = go2w_cmd_vel_bridge.bridge_node:main"]},
)
