from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'forklift_control'

setup(
    name=package_name,
    version='1.0.0',
    # 自动发现包
    packages=find_packages(exclude=['test']),
    data_files=[
        # 注册到 ament 资源索引
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件（如有）
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Forklift Developer',
    maintainer_email='forklift@example.com',
    description='叉车控制包：任务状态机、货叉控制器和手动控制节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 任务状态机节点
            'forklift_task_manager = forklift_control.forklift_task_manager:main',
            # 货叉控制节点
            'fork_controller = forklift_control.fork_controller:main',
            # 手动控制节点
            'manual_controller = forklift_control.manual_controller:main',
        ],
    },
)
