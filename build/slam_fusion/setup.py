from setuptools import find_packages, setup

package_name = 'slam_fusion'

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
    description='SLAM sensor fusion and TF broadcasting',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster = slam_fusion.static_tf_broadcaster:main',
        ],
    },
)
