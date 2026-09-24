import glob

from setuptools import find_packages, setup

package_name = 'mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'mapping/config.yml']),
        ('share/' + package_name + '/Simulation/Base_Images',
            glob.glob('Simulation/Base_Images/*.png')),
        ('share/' + package_name + '/models',
            glob.glob('mapping/models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felipe',
    maintainer_email='felipebsalmon@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mangalarga = mapping.mangalarga:main',
        ],
    },
)
