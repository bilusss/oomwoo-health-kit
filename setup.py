from glob import glob

from setuptools import find_packages, setup

package_name = "oomwoo_health_kit"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lukasz Bilski",
    maintainer_email="102826630+bilusss@users.noreply.github.com",
    description="Verification kit for the OOMWOO health-monitor RFC.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mcu_standin_node = oomwoo_health_kit.mcu_standin_node:main",
            "fake_component_node = oomwoo_health_kit.fake_component_node:main",
            "roster_publisher_node = oomwoo_health_kit.roster_publisher_node:main",
            "topic_heartbeat = oomwoo_health_kit.topic_heartbeat_node:main",
            "job_roster = oomwoo_health_kit.job_roster_node:main",
        ],
    },
)
