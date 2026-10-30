from setuptools import setup

package_name = "agv_vision"

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
    description="카메라 영상 처리 — 라인 검출 (색 마커·AI 분류는 다음 단계)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "line_node = agv_vision.line_node:main",
        ],
    },
)
