from glob import glob
from setuptools import find_packages, setup


package_name = "camera_extrinsic_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="single_fr3_rviz maintainers",
    maintainer_email="maintainer@example.com",
    description="Multi-ArUco camera-to-FR3-base extrinsic calibration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_extrinsic_calibrator = "
            "camera_extrinsic_calibration.calibrator_node:main",
        ],
    },
)
