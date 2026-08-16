#!/usr/bin/env python3
"""PapillArray 接触处理 ROS 2 包安装配置。"""

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "papillarray_contact_processing"

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
    install_requires=["numpy", "setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="langxin11",
    maintainer_email="2658327508@qq.com",
    description="PapillArray 接触信号 bridge、原始数据记录与离线噪声分析工具",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "contact_bridge = papillarray_contact_processing.contact_bridge_node:main",
            "csv_recorder = papillarray_contact_processing.csv_recorder:main",
            "noise_analysis = papillarray_contact_processing.noise_analysis:main",
        ],
    },
)
