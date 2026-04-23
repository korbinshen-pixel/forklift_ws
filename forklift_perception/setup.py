from setuptools import setup

package_name = 'forklift_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='forklift_dev',
    maintainer_email='forklift@example.com',
    description='叉车仿真感知模块',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pallet_detector_node = forklift_perception.pallet_detector_node:main',
            'point_cloud_processor = forklift_perception.point_cloud_processor:main',
        ],
    },
)
