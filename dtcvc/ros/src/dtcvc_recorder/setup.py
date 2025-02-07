from setuptools import find_packages, setup

package_name = "dtcvc_recorder"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="test@test.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dtcvc_recorder_node = dtcvc_recorder.dtcvc_recorder_node:main",
            "dtcvc_playback_node = dtcvc_recorder.dtcvc_playback_node:main",
            "dtcvc_video_to_ros_node = dtcvc_recorder.dtcvc_video_to_ros_node:main",
        ],
    },
)
