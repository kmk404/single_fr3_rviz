from setuptools import setup

name = "omega7_fresh"
setup(
    name=name,
    version="0.1.0",
    packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
        ("share/" + name + "/launch", ["launch/omega7_servo_teleop.launch.py"]),
        ("share/" + name + "/config", ["config/omega7_fresh.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["teleop = omega7_fresh.node:main", "keyboard = omega7_fresh.keyboard:main"]},
)
