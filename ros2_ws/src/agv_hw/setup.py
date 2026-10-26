from setuptools import setup

package_name = "agv_hw"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".drivers"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="하드웨어 드라이버(모터·서보·초음파·LED)와 하드웨어 노드",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_node = agv_hw.motor_node:main",
            "pantilt_node = agv_hw.pantilt_node:main",
            "ultrasonic_node = agv_hw.ultrasonic_node:main",
            "led_node = agv_hw.led_node:main",
        ],
    },
)
