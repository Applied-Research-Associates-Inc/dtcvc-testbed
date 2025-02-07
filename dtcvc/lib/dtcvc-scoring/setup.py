from setuptools import find_packages, setup

package_name = "triage_scorer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=["triage_scorer"]),
    description="Triage Scoring Library",
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
)
