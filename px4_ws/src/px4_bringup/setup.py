from setuptools import setup
import os
from glob import glob


package_name = 'px4_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 1. 安装 launch 根目录文件
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
        # 2. 安装 launch/include 目录文件
        (os.path.join('share', package_name, 'launch/include'), 
        glob('launch/include/*')),
        # 3. 安装 config 目录文件
        (os.path.join('share', package_name, 'config'), 
        glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={
        "test": ["pytest"],
    },
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='863237849@qq.com',
    description='Px4 bringup package including launch files for MAVROS and sensors',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
