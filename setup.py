from setuptools import find_packages, setup

package_name = 'lidar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='nmcilwee@unr.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_lidar = lidar_pkg.lidar_publisher:main',
            'sub_lidar = lidar_pkg.find_gap:main',
            'lidar_node = lidar_pkg.lidar_node:main',
        ],
    },
)
