from setuptools import setup

package_name = "agv_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="주행 제어 drive_node (상태 머신 mission_node는 다음 단계)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_node = agv_control.drive_node:main",
        ],
    },
)
