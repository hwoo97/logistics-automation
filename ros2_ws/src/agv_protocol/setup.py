from setuptools import setup

package_name = "agv_protocol"

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
    description="BT 메시지·영상 프레임·공통 타입 (순수 Python, 노트북 앱과 공용)",
    license="MIT",
    tests_require=["pytest"],
)
