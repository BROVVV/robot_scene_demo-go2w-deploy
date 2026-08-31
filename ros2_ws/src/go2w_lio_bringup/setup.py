from setuptools import find_packages, setup

package_name = "go2w_lio_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/lio.launch.py",
                "launch/point_lio.launch.py",
                "launch/wheel_odom.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="robot_scene_demo maintainers",
    maintainer_email="maintainer@example.invalid",
    description="Fail-closed LIO bringup and isolated Point-LIO bridge for Go2-W",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "output_adapter = go2w_lio_bringup.output_adapter:main",
            "point_lio_bridge = go2w_lio_bringup.point_lio_bridge:main",
            "go2w_wheel_odom = go2w_lio_bringup.wheel_odom:main",
        ]
    },
)
