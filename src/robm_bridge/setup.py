from setuptools import setup

package_name = 'robm_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent Drevelle',
    maintainer_email='vincent.drevelle@univ-rennes1.fr',
    description='Bridges to connect to the M5Stick Rover for ROBM robotics labs at ISTIC (University of Rennes)',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zenoh_bridge = robm_bridge.zenoh_bridge:main',
            'udp_bridge = robm_bridge.udp_bridge:main',
        ],
    },
)
