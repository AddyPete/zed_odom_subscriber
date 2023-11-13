from setuptools import find_packages, setup

package_name = "zed_odom_subscriber"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/waypoint_follow_launch.py"]),
        ("share/" + package_name + "/config", ["config/joystick.yaml"]),
        ("share/" + package_name + "/config", ["config/twist_mux.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minegearscsu",
    maintainer_email="aprlagare1999@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "zed_odom_subscriber_node = zed_odom_subscriber.zed_odom_subscriber_node:main"
        ],
    },
)
