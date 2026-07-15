from setuptools import find_packages, setup

package_name = 'precision_landing'

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
    maintainer='pedro-bevilaqua',
    maintainer_email='pedrobevilaqua04@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mangalarga = precision_landing.mangalarga:main',
            'view_camera = precision_landing.utils.view_camera:main',
        ],
    },
)
