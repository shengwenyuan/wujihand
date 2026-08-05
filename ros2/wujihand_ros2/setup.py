from setuptools import find_packages, setup


PACKAGE_NAME = "wujihand_ros2"


setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (
            f"share/{PACKAGE_NAME}/launch",
            ["launch/dual_teleoperation.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WujiHand maintainers",
    maintainer_email="maintainers@wujihand.invalid",
    description="ROS 2 Jazzy adapters for Wuji dual teleoperation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vive_source = wujihand_ros2.nodes.vive_source:main",
            "glove_source = wujihand_ros2.nodes.glove_source:main",
            (
                "lifecycle_activate = "
                "wujihand_ros2.lifecycle_activate:main"
            ),
        ],
    },
)
