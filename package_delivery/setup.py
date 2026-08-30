import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'package_delivery'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ana-machado',
    maintainer_email='ana.machado8002@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mangalarga = package_delivery.mangalarga:main',
            'collect_sim_photos = package_delivery.collect_sim_photos:main',
            'test_servo = package_delivery.test_servo:main',
            'test_wait = package_delivery.test_wait:main',
        ],
    },
)