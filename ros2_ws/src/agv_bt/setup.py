from setuptools import setup

package_name = "agv_bt"

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
    description="블루투스 RFCOMM 브리지 (노트북 앱 ↔ ROS 토픽)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["bt_bridge_node = agv_bt.bt_bridge_node:main"]},
)
