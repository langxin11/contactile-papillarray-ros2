#!/usr/bin/env python3
"""PapillArray 滑动处理 ROS 2 包安装配置。"""

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "papillarray_slip_processing"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README_zh.md"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="langxin11",
    maintainer_email="2658327508@qq.com",
    description="PapillArray 按需滑动检测状态提取",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "slip_bridge = papillarray_slip_processing.slip_node:main",
        ],
    },
)
