from setuptools import find_packages, setup

package_name = 'slam_landmarks'

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
    description='YOLO-based landmark detection and 3D association for SLAM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = slam_landmarks.yolo_detector_node:main',
            'landmark_3d_node = slam_landmarks.landmark_3d_node:main',
        ],
    },
)
