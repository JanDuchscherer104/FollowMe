from setuptools import setup

package_name = "follow_meB"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jd",
    maintainer_email="jan.duchscherer@hm.edu",
    description="Follow Me ROS2 foxy package. It uttilizes Mediapipe to detect a human and folle it.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follow_me_detector = follow_me.follow_me_detector:main",
            "follow_me_commander = follow_me.follow_me_commander:main",
        ],
    },
)
