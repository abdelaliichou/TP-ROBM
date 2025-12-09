from setuptools import find_packages, setup

package_name = 'robm_tp4_odom'

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
    maintainer='abdelaliichou',
    maintainer_email='abdelaliichou200@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'calib_imu = robm_tp4_odom.calib_imu:main',
            'odometry = robm_tp4_odom.odometry:main',
        ],
    },
)
