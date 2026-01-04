from setuptools import find_packages, setup

package_name = 'slam_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bala',
    maintainer_email='balamahendrareddybhi@gmail.com',
    description='SLAM sensor nodes for Pi AI Camera, Arducam ToF, and MPU6050 IMU module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050_node = slam_sensors.mpu6050_node:main',
            'arducam_tof_node = slam_sensors.arducam_tof_node:main',
            'pi_ai_cam_node = slam_sensors.pi_ai_cam_node:main',
        ],
    },
)
