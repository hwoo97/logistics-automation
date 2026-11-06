from setuptools import setup

package_name = "agv_vision"

setup(
    name=package_name,
    version="0.4.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="카메라 영상 처리 — 라인 검출, 정지선 판정, 색 마커 판별, 표지판 분류(LiteRT)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "line_node = agv_vision.line_node:main",
            "marker_node = agv_vision.marker_node:main",
            "classifier_node = agv_vision.classifier_node:main",
        ],
    },
)
