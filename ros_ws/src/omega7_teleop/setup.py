from setuptools import find_packages, setup


package_name = "omega7_teleop"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/omega7_teleop.yaml"]),
        ("share/" + package_name + "/launch", ["launch/omega7_teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="single_fr3_rviz maintainers",
    maintainer_email="maintainer@example.com",
    description="Safe Cartesian teleoperation of one Franka FR3 with an Omega.7.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "omega7_teleop_node = omega7_teleop.teleop_node:main",
        ],
    },
)
