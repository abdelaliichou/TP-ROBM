from setuptools import find_packages, setup

package_name = 'robm_tp2_move'

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
            'avance_motor = robm_tp2_move.avance_motor:main',
            'gauche_motor = robm_tp2_move.gauche_motor:main',
            'tourne_motor = robm_tp2_move.tourne_motor:main',
            'back_motor = robm_tp2_move.back_motor:main',
            'droit_motor = robm_tp2_move.droit_motor:main',
            'dynamic_motor = robm_tp2_move.dynamic_motor:main',  
        ],
    },
)
