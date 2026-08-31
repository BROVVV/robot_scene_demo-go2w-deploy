from setuptools import find_packages, setup

package_name = "go2w_lidar_preprocessor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/lidar_preprocessor.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="robot_scene_demo maintainers",
    maintainer_email="maintainer@example.invalid",
    description="TF-gated Go2-W LiDAR preprocessor",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lidar_preprocessor = go2w_lidar_preprocessor.preprocessor_node:main",
            "hesai_pandarxt16_preprocessor = "
            "go2w_lidar_preprocessor.hesai_pandarxt16_preprocessor:main",
        ]
    },
)
