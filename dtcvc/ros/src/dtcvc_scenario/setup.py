from setuptools import find_packages, setup
from glob import glob

package_name = "dtcvc_scenario"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (F"lib/{package_name}/dtc_manager/", glob(F"{package_name}/dtc_manager/*.py"))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rosdev",
    maintainer_email="kamartin@ara.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["dtcvc_scenario_node = dtcvc_scenario.dtcvc_scenario_node:main"],
    },
)
